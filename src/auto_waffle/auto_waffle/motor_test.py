#!/usr/bin/env python3
import time
import serial
import serial.tools.list_ports
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class MotorTester:
    def __init__(self):
        # Initialize the motor HAT
        self.motor_hat = Adafruit_MotorHAT(addr=0x60)
        
        # Get motor instances - using M3 for left and M2 for right
        self.left_motor = self.motor_hat.getMotor(3)  # M3
        self.right_motor = self.motor_hat.getMotor(2)  # M2
        
        # Test parameters
        self.test_duration = 3.0  # Duration for each test in seconds
        self.pause_duration = 2.0  # Pause between tests
        
        # Arduino setup
        self.ser = None
        self.arduino_baudrate = 9600
        self.encoder1_count = 0  # Left motor encoder
        self.encoder2_count = 0  # Right motor encoder
        self.connect_to_arduino()
        
    def connect_to_arduino(self):
        """Connect to Arduino for encoder readings"""
        print("Connecting to Arduino...")
        try:
            # Try common Arduino ports
            ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
            for port in ports:
                try:
                    self.ser = serial.Serial(port, self.arduino_baudrate, timeout=1)
                    time.sleep(2)  # Wait for Arduino to reset
                    print(f"Connected to Arduino on {port}")
                    return
                except (serial.SerialException, OSError):
                    continue
            if not self.ser:
                print("Could not connect to Arduino. Encoder readings will not be available.")
        except Exception as e:
            print(f"Error connecting to Arduino: {str(e)}")
            
    def read_encoders(self):
        """Read encoder values from Arduino"""
        if not self.ser:
            return None, None
            
        try:
            # Clear any pending data
            while self.ser.in_waiting:
                self.ser.readline()
                
            # Wait for a valid reading
            start_time = time.time()
            while (time.time() - start_time) < 1.0:  # Timeout after 1 second
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line and "|" in line and "Motor 1:" in line and "Motor 2:" in line:
                        # Parse encoder counts
                        parts = [part.strip() for part in line.split('|')]
                        for part in parts:
                            if "Motor 1:" in part:
                                self.encoder1_count = int(part.replace("Motor 1:", "").strip())
                            elif "Motor 2:" in part:
                                self.encoder2_count = int(part.replace("Motor 2:", "").strip())
                        return self.encoder1_count, self.encoder2_count
            return None, None
        except Exception as e:
            print(f"Error reading encoders: {str(e)}")
            return None, None
            
    def stop_motors(self):
        """Stop and release both motors"""
        if self.left_motor:
            self.left_motor.setSpeed(0)
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        if self.right_motor:
            self.right_motor.setSpeed(0)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            
    def test_single_motor(self, motor, motor_name, pwm):
        """Test a single motor with given PWM value"""
        direction = Adafruit_MotorHAT.FORWARD if pwm >= 0 else Adafruit_MotorHAT.BACKWARD
        abs_pwm = abs(pwm)
        
        print(f"\nTesting {motor_name} motor with PWM: {pwm}")
        print(f"Direction: {'FORWARD' if pwm >= 0 else 'BACKWARD'}")
        print(f"Absolute PWM: {abs_pwm}")
        
        # Read initial encoder values
        start_enc1, start_enc2 = self.read_encoders()
        if start_enc1 is not None and start_enc2 is not None:
            print(f"Initial encoder counts - Left: {start_enc1}, Right: {start_enc2}")
        
        # Set motor speed and direction
        motor.setSpeed(abs_pwm)
        motor.run(direction)
        
        # Run for test duration while monitoring encoders
        start_time = time.time()
        while (time.time() - start_time) < self.test_duration:
            enc1, enc2 = self.read_encoders()
            if enc1 is not None and enc2 is not None:
                if motor_name == "Left":
                    print(f"Left encoder count: {enc1} (Δ: {enc1 - start_enc1 if start_enc1 is not None else 'N/A'})")
                else:
                    print(f"Right encoder count: {enc2} (Δ: {enc2 - start_enc2 if start_enc2 is not None else 'N/A'})")
            time.sleep(0.1)  # Read encoders every 100ms
        
        # Stop motor
        motor.run(Adafruit_MotorHAT.RELEASE)
        
        # Read final encoder values
        end_enc1, end_enc2 = self.read_encoders()
        if end_enc1 is not None and end_enc2 is not None:
            if motor_name == "Left":
                total_ticks = end_enc1 - start_enc1 if start_enc1 is not None else 'N/A'
                print(f"Total encoder ticks (Left): {total_ticks}")
            else:
                total_ticks = end_enc2 - start_enc2 if start_enc2 is not None else 'N/A'
                print(f"Total encoder ticks (Right): {total_ticks}")
                
        print(f"{motor_name} motor test complete")
        
    def run_motor_comparison(self, pwm_value):
        """Run both motors with the same PWM for comparison"""
        print(f"\n=== Testing both motors at PWM {pwm_value} ===")
        
        # Read initial encoder values
        start_enc1, start_enc2 = self.read_encoders()
        if start_enc1 is not None and start_enc2 is not None:
            print(f"Initial encoder counts - Left: {start_enc1}, Right: {start_enc2}")
        
        # Set both motors to the same speed and direction
        direction = Adafruit_MotorHAT.FORWARD if pwm_value >= 0 else Adafruit_MotorHAT.BACKWARD
        abs_pwm = abs(pwm_value)
        
        self.left_motor.setSpeed(abs_pwm)
        self.right_motor.setSpeed(abs_pwm)
        
        # Start both motors simultaneously
        self.left_motor.run(direction)
        self.right_motor.run(direction)
        
        print("Both motors running...")
        
        # Monitor encoders during test
        start_time = time.time()
        while (time.time() - start_time) < self.test_duration:
            enc1, enc2 = self.read_encoders()
            if enc1 is not None and enc2 is not None:
                left_delta = enc1 - start_enc1 if start_enc1 is not None else 'N/A'
                right_delta = enc2 - start_enc2 if start_enc2 is not None else 'N/A'
                print(f"Encoder counts - Left: {enc1} (Δ: {left_delta}), Right: {enc2} (Δ: {right_delta})")
            time.sleep(0.1)  # Read encoders every 100ms
        
        # Stop both motors
        self.stop_motors()
        
        # Read final encoder values
        end_enc1, end_enc2 = self.read_encoders()
        if end_enc1 is not None and end_enc2 is not None:
            left_total = end_enc1 - start_enc1 if start_enc1 is not None else 'N/A'
            right_total = end_enc2 - start_enc2 if start_enc2 is not None else 'N/A'
            print(f"Total encoder ticks - Left: {left_total}, Right: {right_total}")
            if isinstance(left_total, int) and isinstance(right_total, int):
                ratio = left_total / right_total if right_total != 0 else 'N/A'
                print(f"Left/Right ratio: {ratio:.2f}")
        
        print("Test complete")
        
    def run_comprehensive_test(self):
        """Run a series of tests to compare motor behavior"""
        try:
            print("=== Starting Motor Testing Sequence ===")
            
            # Test sequence
            pwm_values = [50, 100, 150, 200, 255]  # Test different PWM values
            
            # Individual motor tests
            for pwm in pwm_values:
                # Test left motor forward
                self.test_single_motor(self.left_motor, "Left", pwm)
                time.sleep(self.pause_duration)
                
                # Test right motor forward
                self.test_single_motor(self.right_motor, "Right", pwm)
                time.sleep(self.pause_duration)
                
                # Test both motors together
                self.run_motor_comparison(pwm)
                time.sleep(self.pause_duration)
                
            # Test reverse direction
            print("\n=== Testing Reverse Direction ===")
            for pwm in [-100, -200]:
                self.run_motor_comparison(pwm)
                time.sleep(self.pause_duration)
            
            print("\n=== Motor Testing Complete ===")
            
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        finally:
            self.stop_motors()
            if self.ser:
                self.ser.close()
            print("Motors stopped and released")

def main():
    tester = MotorTester()
    
    print("Motor Testing Program")
    print("This will run a series of tests to compare motor speeds and responses.")
    print("Please ensure the robot is safely elevated so the wheels can spin freely.")
    print("Press Ctrl+C at any time to stop the test.")
    input("Press Enter to begin testing...")
    
    tester.run_comprehensive_test()

if __name__ == '__main__':
    main() 