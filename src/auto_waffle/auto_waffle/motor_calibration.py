#!/usr/bin/env python3
import time
import serial
import serial.tools.list_ports
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero
        if dt <= 0:
            return 0
            
        # Calculate derivative (rate of change of error)
        derivative = (error - self.prev_error) / dt
        
        # Update integral (accumulated error)
        self.integral += error * dt
        
        # Calculate PID output
        output = (self.kp * error +           # Proportional term
                 self.ki * self.integral +     # Integral term
                 self.kd * derivative)         # Derivative term
        
        # Store values for next iteration
        self.prev_error = error
        self.last_time = current_time
        
        return output
        
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

class MotorCalibrator:
    def __init__(self):
        # Initialize the motor HAT
        self.motor_hat = Adafruit_MotorHAT(addr=0x60)
        
        # Get motor instances - using M3 for left and M2 for right
        self.left_motor = self.motor_hat.getMotor(3)  # M3
        self.right_motor = self.motor_hat.getMotor(2)  # M2
        
        # Arduino setup
        self.ser = None
        self.arduino_baudrate = 9600
        self.encoder1_count = 0  # Left motor encoder
        self.encoder2_count = 0  # Right motor encoder
        
        # Data collection for analysis
        self.left_speeds = []
        self.right_speeds = []
        self.left_pwms = []
        self.right_pwms = []
        self.speed_ratios = []
        self.timestamps = []
        
        # PID Controllers - one for each motor
        # Start with very conservative values
        self.left_pid = PIDController(kp=0.03, ki=0.001, kd=0.01)
        self.right_pid = PIDController(kp=0.03, ki=0.001, kd=0.01)
        
        # Motor parameters
        self.target_speed = 500  # Target speed in encoder ticks per second
        self.min_pwm = 40       # Minimum PWM to move motors
        self.max_pwm = 150      # Maximum PWM to prevent too high speeds
        self.max_pwm_change = 5  # Maximum PWM change per update
        self.motor_ratio = 1.0   # Ratio between motors (will be calibrated)
        
    def connect_to_arduino(self):
        """Connect to Arduino for encoder readings"""
        # Close existing connection if any
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            
        print("Connecting to Arduino...")
        try:
            ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
            for port in ports:
                try:
                    print(f"Trying port {port}...")
                    self.ser = serial.Serial(port, self.arduino_baudrate, timeout=1)
                    time.sleep(2)  # Wait for Arduino to reset
                    
                    # Clear any pending data
                    while self.ser.in_waiting:
                        self.ser.readline()
                    
                    # Send test message
                    self.ser.write(b'ROS:HELLO\n')
                    time.sleep(0.1)
                    
                    # Try to read some data
                    start_time = time.time()
                    while (time.time() - start_time) < 2.0:  # Wait up to 2 seconds for valid data
                        if self.ser.in_waiting:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if "Motor 1:" in line and "Motor 2:" in line:
                                print(f"Successfully connected to Arduino on {port}")
                                return True
                    
                    # If we get here, connection failed
                    self.ser.close()
                    self.ser = None
                except Exception as e:
                    print(f"Failed to connect to {port}: {str(e)}")
                    if self.ser:
                        self.ser.close()
                        self.ser = None
                    continue
                    
            print("Could not connect to Arduino. Calibration cannot proceed.")
            return False
        except Exception as e:
            print(f"Error connecting to Arduino: {str(e)}")
            return False
            
    def read_encoders(self):
        """Read encoder values from Arduino"""
        if not self.ser:
            return None, None
            
        try:
            # Wait for data to be available
            if not self.ser.in_waiting:
                time.sleep(0.1)  # Short delay if no data
                return None, None
                
            # Read a line and decode it
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except (UnicodeDecodeError, AttributeError):
                return None, None
                
            # Parse the line if it contains encoder data
            if line and "|" in line and "Motor 1:" in line and "Motor 2:" in line:
                try:
                    # Split by pipe character and clean up
                    parts = [part.strip() for part in line.split('|')]
                    
                    # Extract encoder counts
                    enc1 = None
                    enc2 = None
                    for part in parts:
                        if "Motor 1:" in part:
                            try:
                                enc1 = int(part.replace("Motor 1:", "").strip())
                            except ValueError:
                                pass
                        elif "Motor 2:" in part:
                            try:
                                enc2 = int(part.replace("Motor 2:", "").strip())
                            except ValueError:
                                pass
                    
                    if enc1 is not None and enc2 is not None:
                        self.encoder1_count = enc1
                        self.encoder2_count = enc2
                        return self.encoder1_count, self.encoder2_count
                except Exception as e:
                    print(f"Error parsing encoder data: {str(e)}")
                    
            return None, None
            
        except Exception as e:
            print(f"Error reading encoders: {str(e)}")
            return None, None
            
    def set_motor_speed(self, motor, target_pwm, direction, current_pwm=None):
        """Set motor speed with ramping and bounds checking"""
        # Ensure PWM is within valid range
        target_pwm = max(0, min(255, int(target_pwm)))
        
        # If no current PWM provided, use target directly
        if current_pwm is None:
            current_pwm = target_pwm
        
        # Calculate allowed PWM change
        if target_pwm > current_pwm:
            new_pwm = min(target_pwm, current_pwm + self.max_pwm_change)
        else:
            new_pwm = max(target_pwm, current_pwm - self.max_pwm_change)
            
        # Apply minimum PWM threshold
        if 0 < new_pwm < self.min_pwm:
            new_pwm = self.min_pwm
            
        # Apply maximum PWM limit
        new_pwm = min(new_pwm, self.max_pwm)
        
        # Set motor speed and direction
        motor.setSpeed(new_pwm)
        motor.run(direction)
        return new_pwm
    
    def measure_motor_ratio(self, pwm=60, duration=10):
        """Run motors at constant PWM to directly measure their ratio"""
        if not self.connect_to_arduino():
            return None
            
        print("\n=== Measuring Motor Speed Ratio ===")
        print(f"PWM value: {pwm}")
        print(f"Duration: {duration} seconds")
        
        # Reset data arrays
        self.left_speeds = []
        self.right_speeds = []
        self.speed_ratios = []
        self.timestamps = []
        
        try:
            # Initialize variables
            start_time = time.time()
            last_time = start_time
            last_enc1, last_enc2 = self.read_encoders()
            
            # Wait up to 2 seconds for valid encoder readings
            retry_count = 0
            while (last_enc1 is None or last_enc2 is None) and retry_count < 20:
                time.sleep(0.1)
                last_enc1, last_enc2 = self.read_encoders()
                retry_count += 1
                
            if last_enc1 is None or last_enc2 is None:
                print("Could not read encoders. Aborting measurement.")
                return None
                
            # Start motors at constant speed
            self.left_motor.setSpeed(pwm)
            self.right_motor.setSpeed(pwm)
            self.left_motor.run(Adafruit_MotorHAT.FORWARD)
            self.right_motor.run(Adafruit_MotorHAT.FORWARD)
            
            print("\nTime  |  Left Speed  |  Right Speed  |  Ratio (R/L)")
            print("-" * 60)
            
            while (time.time() - start_time) < duration:
                # Read current encoder values
                enc1, enc2 = self.read_encoders()
                if enc1 is None or enc2 is None:
                    continue
                    
                current_time = time.time()
                dt = current_time - last_time
                
                if dt >= 0.1:  # Update every 100ms
                    # Calculate speeds
                    left_speed = (enc1 - last_enc1) / dt
                    right_speed = (enc2 - last_enc2) / dt
                    
                    # Calculate ratio
                    if left_speed > 0:
                        ratio = right_speed / left_speed
                    else:
                        ratio = 0
                        
                    # Store data
                    self.left_speeds.append(left_speed)
                    self.right_speeds.append(right_speed)
                    self.speed_ratios.append(ratio)
                    self.timestamps.append(current_time - start_time)
                    
                    # Print status
                    elapsed = current_time - start_time
                    print(f"{elapsed:4.1f}s | {left_speed:10.1f} | {right_speed:11.1f} | {ratio:10.4f}")
                    
                    # Update for next iteration
                    last_time = current_time
                    last_enc1, last_enc2 = enc1, enc2
                    
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
            # Calculate average ratio (excluding first few readings for stabilization)
            if len(self.speed_ratios) > 5:
                avg_ratio = np.mean(self.speed_ratios[5:])
                print(f"\nAverage speed ratio (Right/Left): {avg_ratio:.4f}")
                print(f"This means the left motor needs to be powered at {avg_ratio:.4f}x the right motor's power")
                
                # Store the motor ratio for use in calibration
                self.motor_ratio = avg_ratio
                return avg_ratio
            else:
                print("Not enough data collected for reliable measurement")
                return None
                
        except KeyboardInterrupt:
            print("\nMeasurement interrupted by user")
        finally:
            # Stop motors
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
                
    def calibrate_motors(self, base_pwm=60, duration=15):
        """Run motors with PID control to calibrate speeds"""
        # Ensure Arduino connection
        if not self.connect_to_arduino():
            return
            
        print("\n=== Starting Motor Calibration ===")
        print(f"Target speed: {self.target_speed} ticks/second")
        print(f"Base PWM: {base_pwm}")
        print(f"Duration: {duration} seconds")
        print(f"Using motor ratio: {self.motor_ratio:.4f}")
        
        # Apply initial motor ratio
        left_base_pwm = int(base_pwm * self.motor_ratio)
        right_base_pwm = base_pwm
        
        print(f"Initial PWM values: Left={left_base_pwm}, Right={right_base_pwm}")
        
        # Reset data arrays
        self.left_speeds = []
        self.right_speeds = []
        self.left_pwms = []
        self.right_pwms = []
        self.speed_ratios = []
        self.timestamps = []
        
        try:
            # Initialize variables
            start_time = time.time()
            last_time = start_time
            last_enc1, last_enc2 = self.read_encoders()
            
            # Wait up to 2 seconds for valid encoder readings
            retry_count = 0
            while (last_enc1 is None or last_enc2 is None) and retry_count < 20:
                time.sleep(0.1)
                last_enc1, last_enc2 = self.read_encoders()
                retry_count += 1
                
            if last_enc1 is None or last_enc2 is None:
                print("Could not read encoders. Aborting calibration.")
                return
                
            # Initial motor settings
            left_pwm = left_base_pwm
            right_pwm = right_base_pwm
            
            print("\nTime  |  Left Speed  |  Right Speed  |  Left PWM  |  Right PWM  |  Ratio")
            print("-" * 80)
            
            # Start motors at base speed
            left_pwm = self.set_motor_speed(self.left_motor, left_base_pwm, Adafruit_MotorHAT.FORWARD)
            right_pwm = self.set_motor_speed(self.right_motor, right_base_pwm, Adafruit_MotorHAT.FORWARD)
            time.sleep(0.2)  # Let motors start moving
            
            # Reset PID controllers
            self.left_pid.reset()
            self.right_pid.reset()
            
            while (time.time() - start_time) < duration:
                # Read current encoder values
                enc1, enc2 = self.read_encoders()
                if enc1 is None or enc2 is None:
                    continue
                    
                current_time = time.time()
                dt = current_time - last_time
                
                if dt >= 0.1:  # Update every 100ms
                    # Calculate current speeds (ticks per second)
                    left_speed = (enc1 - last_enc1) / dt
                    right_speed = (enc2 - last_enc2) / dt
                    
                    # Calculate ratio
                    ratio = right_speed / max(1, left_speed)
                    
                    # Store data for analysis
                    self.left_speeds.append(left_speed)
                    self.right_speeds.append(right_speed)
                    self.left_pwms.append(left_pwm)
                    self.right_pwms.append(right_pwm)
                    self.speed_ratios.append(ratio)
                    self.timestamps.append(current_time - start_time)
                    
                    # Calculate errors (difference from target speed)
                    left_error = self.target_speed - left_speed
                    right_error = self.target_speed - right_speed
                    
                    # Get PID outputs
                    left_adjustment = self.left_pid.compute(left_error)
                    right_adjustment = self.right_pid.compute(right_error)
                    
                    # Calculate new PWM values
                    new_left_pwm = left_base_pwm + left_adjustment
                    new_right_pwm = right_base_pwm + right_adjustment
                    
                    # Apply new PWM values with ramping
                    left_pwm = self.set_motor_speed(self.left_motor, new_left_pwm, 
                                                  Adafruit_MotorHAT.FORWARD, left_pwm)
                    right_pwm = self.set_motor_speed(self.right_motor, new_right_pwm, 
                                                   Adafruit_MotorHAT.FORWARD, right_pwm)
                    
                    # Print status
                    elapsed = current_time - start_time
                    print(f"{elapsed:4.1f}s | {left_speed:10.1f} | {right_speed:11.1f} | {left_pwm:9d} | {right_pwm:10d} | {ratio:6.4f}")
                    
                    # Store values for next iteration
                    last_time = current_time
                    last_enc1, last_enc2 = enc1, enc2
                    
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
            print("\nCalibration completed!")
            self.analyze_calibration_results()
                
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user")
        finally:
            # Stop motors
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            if self.ser:
                self.ser.close()
                self.ser = None
            print("\nMotors stopped and released")
    
    def analyze_calibration_results(self):
        """Analyze calibration data and provide recommendations"""
        if len(self.speed_ratios) < 5:
            print("Not enough data for analysis")
            return
            
        # Calculate average speed ratio after stabilization (skip first 5 samples)
        avg_ratio = np.mean(self.speed_ratios[5:])
        std_ratio = np.std(self.speed_ratios[5:])
        
        print("\n=== Calibration Analysis ===")
        print(f"Average speed ratio (Right/Left): {avg_ratio:.4f}")
        print(f"Standard deviation: {std_ratio:.4f}")
        
        # Calculate effective motor multiplier
        if avg_ratio > 1:
            # Right motor is faster, so boost left motor
            motor_multiplier = avg_ratio
            print(f"The left motor should be powered at {motor_multiplier:.4f}x to match the right motor")
            print(f"Suggested motor_factor for waffle_bot.py: left_motor_factor = {motor_multiplier:.4f}")
        else:
            # Left motor is faster, so boost right motor
            motor_multiplier = 1 / avg_ratio
            print(f"The right motor should be powered at {motor_multiplier:.4f}x to match the left motor")
            print(f"Suggested change for waffle_bot.py: swap the motor_factor to right_motor_factor = {motor_multiplier:.4f}")
        
        # Plot speed ratio over time
        try:
            plt.figure(figsize=(10, 6))
            plt.plot(self.timestamps, self.speed_ratios, 'b-')
            plt.axhline(y=avg_ratio, color='r', linestyle='--', label=f'Average: {avg_ratio:.4f}')
            plt.xlabel('Time (s)')
            plt.ylabel('Speed Ratio (Right/Left)')
            plt.title('Motor Speed Ratio During Calibration')
            plt.grid(True)
            plt.legend()
            plt.savefig('motor_ratio_calibration.png')
            print("Saved calibration plot to motor_ratio_calibration.png")
            
            # Plot individual motor speeds
            plt.figure(figsize=(10, 6))
            plt.plot(self.timestamps, self.left_speeds, 'b-', label='Left Motor')
            plt.plot(self.timestamps, self.right_speeds, 'r-', label='Right Motor')
            plt.xlabel('Time (s)')
            plt.ylabel('Speed (ticks/s)')
            plt.title('Motor Speeds During Calibration')
            plt.grid(True)
            plt.legend()
            plt.savefig('motor_speeds_calibration.png')
            print("Saved speeds plot to motor_speeds_calibration.png")
        except Exception as e:
            print(f"Could not create plots: {str(e)}")
            
    def find_optimal_pid(self, duration=30):
        """Run multiple calibration tests with different PID values"""
        print("\n=== Finding Optimal PID Values ===")
        
        # Test different PID combinations
        pid_values = [
            {'kp': 0.01, 'ki': 0.0005, 'kd': 0.005},  # Ultra conservative
            {'kp': 0.02, 'ki': 0.001, 'kd': 0.01},    # Very conservative
            {'kp': 0.04, 'ki': 0.002, 'kd': 0.02},    # Conservative
            {'kp': 0.08, 'ki': 0.004, 'kd': 0.02}     # Moderate
        ]
        
        # Track results
        results = []
        
        # First measure the base motor ratio
        print("\nFirst measuring base motor ratio...")
        self.measure_motor_ratio()
        
        for values in pid_values:
            print(f"\nTesting PID values: Kp={values['kp']}, Ki={values['ki']}, Kd={values['kd']}")
            
            # Update PID controllers with new values
            self.left_pid = PIDController(values['kp'], values['ki'], values['kd'])
            self.right_pid = PIDController(values['kp'], values['ki'], values['kd'])
            
            # Run calibration with these values
            self.calibrate_motors(duration=max(10, duration/len(pid_values)))
            
            # Calculate metrics for this run
            if len(self.speed_ratios) > 5:
                avg_ratio = np.mean(self.speed_ratios[5:])
                std_dev = np.std(self.speed_ratios[5:])
                
                # Calculate error as how far from 1.0 the ratio is, plus the standard deviation
                error = abs(1.0 - avg_ratio) + std_dev
                
                results.append({
                    'values': values.copy(),
                    'avg_ratio': avg_ratio,
                    'std_dev': std_dev,
                    'error': error
                })
                
                print(f"Result: Avg Ratio={avg_ratio:.4f}, StdDev={std_dev:.4f}, Error={error:.4f}")
                
        # Find best result
        if results:
            # Sort by error (lower is better)
            results.sort(key=lambda x: x['error'])
            best = results[0]
            
            print("\n=== Optimal PID Values Found ===")
            print(f"Kp: {best['values']['kp']}")
            print(f"Ki: {best['values']['ki']}")
            print(f"Kd: {best['values']['kd']}")
            print(f"Average Ratio: {best['avg_ratio']:.4f}")
            print(f"Standard Deviation: {best['std_dev']:.4f}")
            print(f"Error: {best['error']:.4f}")
            
            # Set the best values
            self.left_pid = PIDController(**best['values'])
            self.right_pid = PIDController(**best['values'])
            
            return best['values']
        else:
            print("\nNo valid PID values found")
            return None
            
def main():
    calibrator = MotorCalibrator()
    
    print("Motor Calibration Program")
    print("This will run PID-controlled calibration to match motor speeds.")
    print("Please ensure the robot is safely elevated so the wheels can spin freely.")
    print("Press Ctrl+C at any time to stop the calibration.")
    print("\nOptions:")
    print("1. Measure motor speed ratio only")
    print("2. Run basic calibration")
    print("3. Find optimal PID values")
    print("4. Full calibration suite (ratio + PID + final test)")
    
    choice = input("\nEnter your choice (1-4): ")
    
    if choice == "1":
        calibrator.measure_motor_ratio()
    elif choice == "2":
        calibrator.measure_motor_ratio()  # First get the ratio
        calibrator.calibrate_motors()
    elif choice == "3":
        calibrator.find_optimal_pid()
    elif choice == "4":
        # Measure base ratio
        print("\n=== Step 1: Measuring base motor ratio ===")
        calibrator.measure_motor_ratio()
        
        # Find optimal PID
        print("\n=== Step 2: Finding optimal PID values ===")
        best_pid = calibrator.find_optimal_pid(duration=40)
        
        # Final calibration with optimal values
        print("\n=== Step 3: Final calibration with optimal values ===")
        calibrator.calibrate_motors(duration=20)
        
        # Provide final recommendations
        print("\n=== Final Recommendations ===")
        print("For waffle_bot.py, update these values:")
        print(f"left_motor_factor = {calibrator.motor_ratio:.4f}")
        print(f"self.speed_pid = PIDController(kp={best_pid['kp']}, ki={best_pid['ki']}, kd={best_pid['kd']})")
    else:
        print("Invalid choice")

if __name__ == '__main__':
    main() 