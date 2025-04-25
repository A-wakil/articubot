#!/usr/bin/env python3
import time
import serial
import serial.tools.list_ports
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

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
        
        # PID Controllers - one for each motor
        # Start with very conservative values
        self.left_pid = PIDController(kp=0.02, ki=0.001, kd=0.01)
        self.right_pid = PIDController(kp=0.02, ki=0.001, kd=0.01)
        
        # Motor parameters
        self.target_speed = 100  # Target speed in encoder ticks per second
        self.min_pwm = 40       # Minimum PWM to move motors
        self.max_pwm = 120      # Maximum PWM to prevent too high speeds
        self.max_pwm_change = 5  # Maximum PWM change per update
        
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
            
    def calibrate_motors(self, base_pwm=50, duration=10):
        """Run motors with PID control to calibrate speeds"""
        # Ensure Arduino connection
        if not self.connect_to_arduino():
            return
            
        print("\n=== Starting Motor Calibration ===")
        print(f"Target speed: {self.target_speed} ticks/second")
        print(f"Base PWM: {base_pwm}")
        print(f"Duration: {duration} seconds")
        
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
            left_pwm = base_pwm
            right_pwm = base_pwm
            
            print("\nTime  |  Left Speed  |  Right Speed  |  Left PWM  |  Right PWM")
            print("-" * 60)
            
            # Start motors at base speed
            left_pwm = self.set_motor_speed(self.left_motor, base_pwm, Adafruit_MotorHAT.FORWARD)
            right_pwm = self.set_motor_speed(self.right_motor, base_pwm, Adafruit_MotorHAT.FORWARD)
            time.sleep(0.1)  # Let motors start moving
            
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
                    
                    # Calculate errors (difference from target speed)
                    left_error = self.target_speed - left_speed
                    right_error = self.target_speed - right_speed
                    
                    # Get PID outputs
                    left_adjustment = self.left_pid.compute(left_error)
                    right_adjustment = self.right_pid.compute(right_error)
                    
                    # Calculate new PWM values
                    new_left_pwm = base_pwm + left_adjustment
                    new_right_pwm = base_pwm + right_adjustment
                    
                    # Apply new PWM values with ramping
                    left_pwm = self.set_motor_speed(self.left_motor, new_left_pwm, 
                                                  Adafruit_MotorHAT.FORWARD, left_pwm)
                    right_pwm = self.set_motor_speed(self.right_motor, new_right_pwm, 
                                                   Adafruit_MotorHAT.FORWARD, right_pwm)
                    
                    # Print status
                    elapsed = current_time - start_time
                    print(f"{elapsed:4.1f}s | {left_speed:10.1f} | {right_speed:11.1f} | {left_pwm:9d} | {right_pwm:10d}")
                    
                    # Store values for next iteration
                    last_time = current_time
                    last_enc1, last_enc2 = enc1, enc2
                    
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
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
            
    def find_optimal_pid(self, duration=30):
        """Run multiple calibration tests with different PID values"""
        print("\n=== Finding Optimal PID Values ===")
        
        # Test different PID combinations - much more conservative values
        pid_values = [
            {'kp': 0.01, 'ki': 0.0005, 'kd': 0.005},  # Ultra conservative
            {'kp': 0.02, 'ki': 0.001, 'kd': 0.01},    # Very conservative
            {'kp': 0.04, 'ki': 0.002, 'kd': 0.02}     # Conservative
        ]
        
        best_error = float('inf')
        best_pid = None
        
        for values in pid_values:
            print(f"\nTesting PID values: Kp={values['kp']}, Ki={values['ki']}, Kd={values['kd']}")
            
            # Update PID controllers with new values
            self.left_pid = PIDController(values['kp'], values['ki'], values['kd'])
            self.right_pid = PIDController(values['kp'], values['ki'], values['kd'])
            
            # Run calibration with these values
            self.calibrate_motors(duration=duration/len(pid_values))
            
            # Calculate error (you might want to modify this based on your needs)
            # For now, we'll use the final PID errors
            total_error = abs(self.left_pid.prev_error) + abs(self.right_pid.prev_error)
            
            if total_error < best_error:
                best_error = total_error
                best_pid = values
                
        if best_pid:
            print("\n=== Optimal PID Values Found ===")
            print(f"Kp: {best_pid['kp']}")
            print(f"Ki: {best_pid['ki']}")
            print(f"Kd: {best_pid['kd']}")
        else:
            print("\nNo valid PID values found")
            
        return best_pid

def main():
    calibrator = MotorCalibrator()
    
    print("Motor Calibration Program")
    print("This will run PID-controlled calibration to match motor speeds.")
    print("Please ensure the robot is safely elevated so the wheels can spin freely.")
    print("Press Ctrl+C at any time to stop the calibration.")
    print("\nOptions:")
    print("1. Run basic calibration")
    print("2. Find optimal PID values")
    
    choice = input("\nEnter your choice (1 or 2): ")
    
    if choice == "1":
        calibrator.calibrate_motors()
    elif choice == "2":
        best_pid = calibrator.find_optimal_pid()
        print("\nWould you like to run a final calibration with these values? (y/n)")
        if input().lower() == 'y':
            calibrator.left_pid = PIDController(**best_pid)
            calibrator.right_pid = PIDController(**best_pid)
            calibrator.calibrate_motors()
    else:
        print("Invalid choice")

if __name__ == '__main__':
    main() 