import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from transforms3d.euler import euler2quat
import serial
import serial.tools.list_ports
import math
import time
import os
import subprocess
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster

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

class WaffleBotNode(Node):
    def __init__(self):
        super().__init__('waffle_bot_node')
        self.get_logger().info(">>> Initializing WaffleBotNode")

        # Motor control parameters
        self.min_pwm = 80  # Minimum PWM to overcome motor inertia
        self.max_pwm = 255  # Maximum PWM value
        self.pwm_scale = 1.0  # Scale factor for PWM values
        self.max_linear_speed = 0.5  # Max linear speed from teleop (m/s)
        self.max_angular_speed = 0.8  # Max angular speed from teleop (rad/s)
        
        
        self.swap_motors = False  # Set to True to swap M1 and M2 if wired incorrectly
        
        # Fixed calibration factor (calibrated from motor_calibration.py)
        self.left_motor_factor = 1.015  # Calibrated value from tests
        
        # PID Controller for balancing motor speeds
        # Values from calibration tests
        self.speed_pid = PIDController(kp=0.02, ki=0.001, kd=0.01)
        self.last_left_count = 0
        self.last_right_count = 0
        self.last_pid_time = time.time()
        self.pid_enabled = True  # Enable/disable PID control
        self.prev_direction = None  # Track previous movement direction for PID reset
        
        # Test mode flag
        self.test_mode = False  # Set to False to disable automatic motor testing
        self.test_sequence_step = 0
        
        # Add cmd_vel tracking variables
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_count = 0
        
        # Arduino and encoder variables
        self.disable_arduino = False  # Set to True to disable Arduino connection attempts
        self.arduino_baudrate = 9600  # Arduino baud rate - try different values if not working
        
        # Specify the preferred Arduino port - change this to match your Arduino
        # For example: /dev/ttyACM0 or /dev/ttyUSB0
        self.preferred_arduino_port = '/dev/ttyACM0'
        
        # Exclude ports that might be used by other devices like LiDAR
        self.excluded_ports = ['/dev/ttyUSB0']  # Add any ports used by LiDAR here
        
        self.encoder1_count = 0
        self.encoder2_count = 0
        self.last_encoder_display_time = self.get_clock().now()

        # Flag to track if we're shutting down
        self.is_shutting_down = False

        # Initialize motor control
        self.left_motor = None
        self.right_motor = None
        self.motor_hat = None
        if not self.setup_motors():
            self.get_logger().error(">>> Failed to initialize motors!")
            return

        # Register shutdown callback to ensure motors stop
        try:
            import atexit
            atexit.register(self.stop_motors)
        except Exception as e:
            self.get_logger().warning(f">>> Could not register atexit handler: {str(e)}")

        # Initialize odometry (Arduino-dependent)
        self.ser = None
        self.setup_odometry()

        # Create ROS subscribers and publishers with explicit QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create cmd_vel subscription with QoS profile
        try:
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                qos_profile
            )
        except Exception as e:
            self.get_logger().error(f">>> Failed to create cmd_vel subscription: {str(e)}")
            return

        # Create odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
        # Initialize TF broadcaster for odometry transform
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(">>> TF broadcaster initialized for odom→base_link")
        
        # Flag to track if odometry has been processed at least once
        self.odometry_processed = False
        
        # Add timers
        self.last_time = self.get_clock().now()
        if not self.disable_arduino:  # Create Arduino timers if Arduino is enabled
            # Create timer to check for Arduino data at 10Hz
            self.encoder_timer = self.create_timer(0.1, self.update_odometry)
            # Create timer to check Arduino connection at 1Hz
            self.connection_timer = self.create_timer(1.0, self.check_arduino_connection)
        
        # Add a motor check timer
        self.create_timer(1.0, self.check_motor_hat)
        
        # Add dedicated timer to publish transforms at 20Hz regardless of encoder updates
        self.tf_timer = self.create_timer(0.05, self.publish_transforms)
        
        # Add a test motor timer if test_mode is enabled
        if self.test_mode:
            self.get_logger().info(">>> TEST MODE ENABLED - Motors will move automatically every 5 seconds")
            self.create_timer(5.0, self.test_motors_timer)
        
        # Add a cmd_vel diagnostic timer to report subscription status and cmd_vel activity
        # Set to a lower frequency (15 seconds) to reduce log noise
        self.create_timer(15.0, self.check_cmd_vel)
        
        self.get_logger().info(">>> WaffleBotNode ready!")

    def destroy_node(self):
        """Called when the node is being destroyed"""
        self.get_logger().info(">>> Shutting down - stopping motors")
        self.is_shutting_down = True
        self.stop_motors()
        super().destroy_node()
    
    def stop_motors(self):
        """Stop all motors and release them"""
        try:
            # Ensure all motors are stopped
            if self.left_motor:
                self.left_motor.setSpeed(0)
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            
            if self.right_motor:
                self.right_motor.setSpeed(0)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
                
            # Release all motors from the motor HAT
            if self.motor_hat:
                for i in range(1, 5):  # Motor HAT has 4 motor channels
                    try:
                        motor = self.motor_hat.getMotor(i)
                        motor.run(Adafruit_MotorHAT.RELEASE)
                    except Exception as e:
                        pass
                        
            self.get_logger().info(">>> All motors released")
        except Exception as e:
            self.get_logger().error(f">>> Error stopping motors: {str(e)}")

    def check_motor_hat(self):
        """Periodically check Motor HAT connection"""
        try:
            # Try to read from Motor HAT
            if self.motor_hat is None:
                self.get_logger().error(">>> Motor HAT not initialized!")
                self.setup_motors()
                return
            
            # Try to access motors to check connection
            test_motor = self.motor_hat.getMotor(1)
            test_motor.run(Adafruit_MotorHAT.RELEASE)
            # No logging during normal operation
        except Exception as e:
            self.get_logger().error(f">>> Motor HAT check failed: {str(e)}")
            self.get_logger().info(">>> Attempting to reinitialize Motor HAT...")
            self.setup_motors()

    def setup_motors(self):
        """Initialize motor control hardware"""
        self.get_logger().info(">>> Setting up motors...")
        
        try:
            # Release any existing motors
            if self.left_motor:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            if self.right_motor:
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            
            # Create new Motor HAT instance
            self.motor_hat = Adafruit_MotorHAT(addr=0x60)
            
            # Get motor instances - using M3 for left and M2 for right
            self.left_motor = self.motor_hat.getMotor(3)  # M3
            self.right_motor = self.motor_hat.getMotor(2)  # M2
            
            # Brief test for motors to verify they work
            self.get_logger().info(">>> Testing motors...")
            
            # Test left motor (M3)
            self.left_motor.setSpeed(150)
            self.left_motor.run(Adafruit_MotorHAT.FORWARD)
            time.sleep(0.3)
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            
            # Test right motor (M2)
            self.right_motor.setSpeed(100)
            self.right_motor.run(Adafruit_MotorHAT.FORWARD)
            time.sleep(0.3)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            
            self.get_logger().info(">>> Motor setup complete!")
            return True
        except Exception as e:
            self.get_logger().error(f">>> Failed to initialize motors: {str(e)}")
            self.get_logger().error("Please check motor connections and power supply")
            return False

    def setup_odometry(self):
        """Initialize odometry-related components (Arduino)"""
        # Initialize position/orientation state
        self.x = 0.0         # Position x in meters
        self.y = 0.0         # Position y in meters
        self.theta = 0.0     # Orientation in radians
        
        # Robot physical parameters - ensure these match the values in waffle_core.xacro
        self.wheel_radius = 0.04     # Wheel radius in meters (40mm)
        self.wheel_base = 0.3        # Distance between wheels in meters (300mm)
        self.wheel_to_front = 0.06   # Distance from wheel center to front (60mm)
        self.base_width = 0.265      # Base width in meters (265mm)
        self.base_length = 0.265     # Base length in meters (265mm)
        self.base_height = 0.06      # Base height in meters (60mm)
        self.caster_height = 0.014   # Caster slider height in meters (14mm)
        self.ticks_per_rev = 1970    # Encoder ticks per wheel revolution
        
        # Tracking variables
        self.last_encoder1 = 0     # Previous left encoder value
        self.last_encoder2 = 0     # Previous right encoder value
        self.encoder1_count = 0    # Current left encoder value
        self.encoder2_count = 0    # Current right encoder value
        self.odometry_enabled = False  # Track if odometry is enabled
        
        # Skip Arduino connection if disabled
        if self.disable_arduino:
            return

        # Try to connect to Arduino with different baud rates if needed
        self.try_arduino_connection()
        if not self.ser:
            self.get_logger().warning('Could not connect to Arduino. Odometry disabled.')
        else:
            self.odometry_enabled = True
            self.get_logger().info('Odometry enabled with encoder feedback')
            
    def try_arduino_connection(self):
        """Try connecting to Arduino with different baud rates if needed"""
        # List of baud rates to try
        baud_rates = [9600, 115200, 57600, 38400, 19200]
        
        # First try with the configured baud rate
        self.connect_to_arduino()
        
        # If failed, try other common baud rates
        if not self.ser:
            self.get_logger().warning(f"Failed to connect at {self.arduino_baudrate} baud. Trying other rates...")
            original_rate = self.arduino_baudrate
            
            for rate in baud_rates:
                if rate != original_rate:  # Skip the one we already tried
                    self.get_logger().info(f"Trying baud rate: {rate}")
                    self.arduino_baudrate = rate
                    self.connect_to_arduino()
                    if self.ser:
                        self.get_logger().info(f"Successfully connected at {rate} baud!")
                        break
            
            # Restore original rate if all attempts failed
            if not self.ser:
                self.arduino_baudrate = original_rate

    def scale_pwm(self, speed):
        """Scale speed to PWM with minimum threshold"""
        abs_speed = abs(speed)
        if abs_speed < 0.01:  # Dead zone
            # self.get_logger().debug(f">>> Speed {speed} in dead zone, releasing motor")
            return 0, Adafruit_MotorHAT.RELEASE
        
        # Scale speed to PWM, ensuring we're above minimum threshold
        pwm = int(self.min_pwm + (self.max_pwm - self.min_pwm) * abs_speed)
        pwm = min(pwm, self.max_pwm)
        
        direction = Adafruit_MotorHAT.FORWARD if speed >= 0 else Adafruit_MotorHAT.BACKWARD
        # self.get_logger().debug(f">>> Scaled speed {speed} to PWM {pwm}, direction {direction}")
        return pwm, direction

    def test_motors(self):
        """Test both motors at minimum speed"""
        try:
            # Test sequence: forward, stop, backward, stop
            for i, motor in enumerate([self.left_motor, self.right_motor], 1):
                self.get_logger().info(f">>> Testing Motor {i} at minimum PWM: {self.min_pwm}")
                
                # Forward test
                self.get_logger().info(f">>> Motor {i} - Testing FORWARD")
                motor.setSpeed(self.min_pwm)
                motor.run(Adafruit_MotorHAT.FORWARD)
                time.sleep(1.0)
                
                # Stop
                self.get_logger().info(f">>> Motor {i} - Testing STOP")
                motor.run(Adafruit_MotorHAT.RELEASE)
                time.sleep(0.5)
                
                # Backward test
                self.get_logger().info(f">>> Motor {i} - Testing BACKWARD")
                motor.setSpeed(self.min_pwm)
                motor.run(Adafruit_MotorHAT.BACKWARD)
                time.sleep(1.0)
                
                # Final stop
                self.get_logger().info(f">>> Motor {i} - Final STOP")
                motor.run(Adafruit_MotorHAT.RELEASE)
                time.sleep(0.5)
            
            self.get_logger().info(">>> Motor test completed successfully!")
        except Exception as e:
            self.get_logger().error(f">>> Motor test failed: {str(e)}")

    def connect_to_arduino(self):
        """Try to connect to the Arduino by checking available ports."""
        # Use the configured baud rate
        arduino_baud_rate = self.arduino_baudrate
        self.get_logger().info(f"Trying to connect to Arduino at {arduino_baud_rate} baud")
        
        # First try the preferred port if specified
        if self.preferred_arduino_port:
            self.get_logger().info(f"Trying preferred Arduino port: {self.preferred_arduino_port}")
            try:
                # Close existing connection if any
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                # Use a short timeout to avoid blocking
                self.ser = serial.Serial(self.preferred_arduino_port, arduino_baud_rate, timeout=0.5)
                
                # Clear any pending data
                if self.ser.in_waiting:
                    self.ser.read(self.ser.in_waiting)
                
                # Wait for Arduino to reset and initialize
                time.sleep(2.0)
                
                # Send a test message
                self.ser.write(b'ROS:HELLO\n')
                time.sleep(0.1)
                
                self.get_logger().info(f'Connected to Arduino on preferred port {self.preferred_arduino_port} at {arduino_baud_rate} baud')
                return
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"Failed to connect to preferred port {self.preferred_arduino_port}: {str(e)}")
        
        # Common Arduino identifiers
        arduino_ids = [
            'Arduino',
            'USB2.0-Serial',
            'ACM',
            'USB Serial'
        ]
        
        # Get all available ports
        ports = list(serial.tools.list_ports.comports())
        self.get_logger().info(f"Available ports: {[p.device for p in ports]}")
        
        # Filter out excluded ports
        available_ports = [p for p in ports if p.device not in self.excluded_ports]
        if len(available_ports) < len(ports):
            self.get_logger().info(f"Excluded ports: {self.excluded_ports}")
            self.get_logger().info(f"Remaining available ports: {[p.device for p in available_ports]}")
        
        # Try to connect to any remaining port that matches Arduino identifiers
        for port in available_ports:
            # Skip if this is an excluded port
            if port.device in self.excluded_ports:
                continue
                
            # Check if any of the Arduino identifiers are in the port description
            if any(id in port.description for id in arduino_ids):
                try:
                    # Close existing connection if any
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                    
                    # Use a short timeout to avoid blocking
                    self.ser = serial.Serial(port.device, arduino_baud_rate, timeout=0.5)
                    
                    # Clear any pending data
                    if self.ser.in_waiting:
                        self.ser.read(self.ser.in_waiting)
                    
                    # Wait for Arduino to reset and initialize
                    time.sleep(2.0)
                    
                    # Send a test message
                    self.ser.write(b'ROS:HELLO\n')
                    time.sleep(0.1)
                    
                    self.get_logger().info(f'Connected to Arduino on {port.device} at {arduino_baud_rate} baud')
                    return
                except serial.SerialException as e:
                    self.get_logger().warning(f"Failed to connect to {port.device}: {str(e)}")
        
        # If no Arduino found with identifiers, try common device names
        common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB1']
        for port in common_ports:
            # Skip if this is an excluded port
            if port in self.excluded_ports:
                continue
                
            if port not in [p.device for p in ports]:  # Skip if already checked
                try:
                    # Close existing connection if any
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                    
                    self.ser = serial.Serial(port, arduino_baud_rate, timeout=0.5)
                    
                    # Clear any pending data
                    if self.ser.in_waiting:
                        self.ser.read(self.ser.in_waiting)
                    
                    # Wait for Arduino to reset
                    time.sleep(2.0)
                    
                    # Send a test message
                    self.ser.write(b'ROS:HELLO\n')
                    time.sleep(0.1)
                    
                    self.get_logger().info(f'Connected to Arduino on {port} at {arduino_baud_rate} baud')
                    return
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warning(f"Failed to connect to {port}: {str(e)}")
        
        # No Arduino found
        self.ser = None
        self.get_logger().warning("No Arduino found on any port")

    def check_arduino_connection(self):
        """Periodically check if Arduino is still connected."""
        try:
            if not self.ser or not self.ser.is_open:
                self.get_logger().info("Arduino disconnected. Attempting to reconnect...")
                self.connect_to_arduino()
                if self.ser:
                    self.get_logger().info("Arduino reconnected successfully!")
            else:
                # Send a ping every few seconds to make sure connection is alive
                self.ser.write(b'ROS:PING\n')
                
                # Check if we can read from the Arduino
                try:
                    # If there's no data for a while, the connection might be broken
                    if self.ser.in_waiting == 0:
                        self.get_logger().info("No data from Arduino - sending test message")
                        self.ser.write(b'ROS:TEST\n')
                except Exception as e:
                    self.get_logger().warning(f"Arduino connection error: {str(e)}")
                    if self.ser:
                        try:
                            self.ser.close()
                        except:
                            pass
                    self.ser = None
                    self.connect_to_arduino()
        except Exception as e:
            self.get_logger().warning(f"Error checking Arduino connection: {str(e)}")
            # Try to clean up and reconnect
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
            self.ser = None

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages"""
        # Update tracking variables
        self.cmd_vel_count += 1
        self.last_cmd_vel_time = self.get_clock().now()
        
        try:
            # Calculate base PWM values
            left_pwm = 0
            right_pwm = 0
            
            # Handle linear velocity (forward/backward)
            if abs(msg.linear.x) > 0.01:
                # Calculate linear PWM based on percentage of max_linear_speed
                # Scale from 0-max_linear_speed to min_pwm-max_pwm
                linear_percent = abs(msg.linear.x) / self.max_linear_speed
                base_pwm = int(self.min_pwm + (self.max_pwm - self.min_pwm) * linear_percent)
                base_pwm = min(base_pwm, self.max_pwm)  # Cap at max_pwm
                
                self.get_logger().info(f">>> Linear: {msg.linear.x}/{self.max_linear_speed} = {linear_percent:.2f} -> PWM {base_pwm}")
                
                if msg.linear.x > 0:  # Forward
                    left_pwm = base_pwm
                    right_pwm = base_pwm
                else:  # Backward
                    left_pwm = -base_pwm
                    right_pwm = -base_pwm
            
            # Add angular velocity (turning)
            if abs(msg.angular.z) > 0.01:
                # Calculate turn intensity based on percentage of max_angular_speed
                # Scale from 0-max_angular_speed to 0-50% of base_pwm
                angular_percent = abs(msg.angular.z) / self.max_angular_speed
                # Use either a percentage of the base PWM or a minimum value to ensure turning works
                turn_pwm = int(max(20, abs(base_pwm if 'base_pwm' in locals() else self.min_pwm) * 0.5 * angular_percent))
                
                self.get_logger().info(f">>> Angular: {msg.angular.z}/{self.max_angular_speed} = {angular_percent:.2f} -> PWM {turn_pwm}")
                
                if msg.angular.z > 0:  # Left turn
                    left_pwm -= turn_pwm
                    right_pwm += turn_pwm
                else:  # Right turn
                    left_pwm += turn_pwm
                    right_pwm -= turn_pwm
            
            # Ensure values are within range
            left_pwm = max(min(left_pwm, self.max_pwm), -self.max_pwm)
            right_pwm = max(min(right_pwm, self.max_pwm), -self.max_pwm)
            
            # Boost very small commands to overcome motor inertia
            if 0 < abs(left_pwm) < self.min_pwm:
                left_pwm = self.min_pwm if left_pwm > 0 else -self.min_pwm
            if 0 < abs(right_pwm) < self.min_pwm:
                right_pwm = self.min_pwm if right_pwm > 0 else -self.min_pwm
            
            # Swap motors if configured
            if self.swap_motors:
                left_pwm, right_pwm = right_pwm, left_pwm
                
            # Log and send motor commands for ALL incoming messages, including zeros
            # This ensures the robot stops when zeros are sent
            self.get_logger().info(f">>> MOTORS - Left: {left_pwm}, Right: {right_pwm} (angular: {msg.angular.z}, linear: {msg.linear.x})")
            self.direct_motor_control(left_pwm, right_pwm)

        except Exception as e:
            self.get_logger().error(f'>>> Error in cmd_vel_callback: {str(e)}')
            import traceback
            self.get_logger().error(f'>>> Stack trace: {traceback.format_exc()}')

    def update_odometry(self):
        if not self.ser:
            return

        try:
            # Check if there's data available from Arduino
            if self.ser.in_waiting:
                try:
                    # Read a line from the Arduino
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Process meaningful lines only
                    if line and "|" in line:
                        # Look for lines that have the motor count format
                        if "Motor 1:" in line and "Motor 2:" in line:
                            # Shows the raw data occasionally for debugging
                            now = self.get_clock().now()
                            time_since_last = (now - self.last_encoder_display_time).nanoseconds / 1e9
                            if time_since_last >= 5.0:  # Only log raw data every 5 seconds
                                self.get_logger().info(f"RAW: '{line}'")
                                
                            # Parse the counts using the format from our test script
                            # Format example: | 355s | Motor 1: 19794 | Motor 2: 20386 |
                            try:
                                # Split by pipe character and clean up
                                parts = [part.strip() for part in line.split('|')]
                                
                                # Extract Motor 1 count (should be in part at index 2)
                                prev_encoder1 = self.encoder1_count
                                for part in parts:
                                    if "Motor 1:" in part:
                                        motor1_str = part.replace("Motor 1:", "").strip()
                                        try:
                                            self.encoder1_count = int(motor1_str)
                                        except ValueError:
                                            pass
                                
                                # Extract Motor 2 count (should be in part at index 3)
                                prev_encoder2 = self.encoder2_count
                                for part in parts:
                                    if "Motor 2:" in part:
                                        motor2_str = part.replace("Motor 2:", "").strip()
                                        try:
                                            self.encoder2_count = int(motor2_str)
                                        except ValueError:
                                            pass
                                
                                # Display encoder counts at regular intervals
                                self.display_encoder_data()
                                
                                # Run PID calculation to adjust for speed differences
                                if self.pid_enabled:
                                    self.update_motor_pid(prev_encoder1, prev_encoder2)
                                
                                # Calculate and publish odometry
                                self.calculate_and_publish_odometry(prev_encoder1, prev_encoder2)
                                
                            except Exception as e:
                                if time_since_last >= 5.0:  # Log errors less frequently
                                    self.get_logger().warning(f"Error parsing Arduino data: {str(e)}")

                except Exception as e:
                    # Only log occasional errors to avoid flooding
                    now = self.get_clock().now()
                    time_since_last = (now - self.last_encoder_display_time).nanoseconds / 1e9
                    if time_since_last >= 5.0:
                        self.get_logger().warning(f"Error reading from Arduino: {str(e)}")
        except Exception as e:
            # Only log occasional errors to avoid flooding
            now = self.get_clock().now()
            time_since_last = (now - self.last_encoder_display_time).nanoseconds / 1e9
            if time_since_last >= 5.0:
                self.get_logger().warning(f"Error in update_odometry: {str(e)}")
            
    def calculate_and_publish_odometry(self, prev_encoder1, prev_encoder2):
        """Calculate and publish odometry based on encoder changes"""
        # Calculate encoder ticks change
        d_encoder1 = self.encoder1_count - prev_encoder1
        d_encoder2 = self.encoder2_count - prev_encoder2
        
        # Get current time
        current_time = self.get_clock().now()
        
        # If there are encoder changes, update the odometry
        if d_encoder1 != 0 or d_encoder2 != 0:
            # Calculate time difference
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            
            # Skip if time difference is too small to avoid division by zero
            if dt >= 0.0001:
                # Convert encoder ticks to distance traveled by each wheel
                left_dist = (2 * math.pi * self.wheel_radius) * (d_encoder1 / self.ticks_per_rev)
                right_dist = (2 * math.pi * self.wheel_radius) * (d_encoder2 / self.ticks_per_rev)
                
                # Calculate robot's linear and angular displacement
                dx = (left_dist + right_dist) / 2.0
                dtheta = (right_dist - left_dist) / self.wheel_base
                
                # Update position and orientation
                self.theta += dtheta
                self.x += dx * math.cos(self.theta)
                self.y += dx * math.sin(self.theta)
                
                # Set velocities for odometry message
                linear_vel = dx / dt
                angular_vel = dtheta / dt
                
                # Mark that we've processed odometry at least once
                self.odometry_processed = True
            else:
                # Use zero velocities if dt is too small
                linear_vel = 0.0
                angular_vel = 0.0
        else:
            # No encoder changes, zero velocities
            linear_vel = 0.0
            angular_vel = 0.0
            
        # Create and publish odometry message (always do this)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        q = euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        
        # Set velocities
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Log odometry occasionally
        time_since_last = (current_time - self.last_encoder_display_time).nanoseconds / 1e9
        if time_since_last >= 5.0:
            self.get_logger().info(f"Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
            self.last_encoder_display_time = current_time

    def display_encoder_data(self):
        """Display encoder readings at a reasonable frequency"""
        now = self.get_clock().now()
        time_since_last = (now - self.last_encoder_display_time).nanoseconds / 1e9
        
        # Only display encoder data at most once per second to avoid flooding the console
        if time_since_last >= 1.0:
            self.get_logger().info(f"Encoder counts - Left: {self.encoder1_count}, Right: {self.encoder2_count}")
            self.last_encoder_display_time = now

    def test_motors_timer(self):
        """Periodic test of motors regardless of cmd_vel input"""
        self.get_logger().info("\n>>> AUTOMATIC MOTOR TEST SEQUENCE")
        
        # Simple test sequence: forward, stop, backward, stop, turn left, stop, turn right, stop
        test_sequences = [
            (50, 50),    # Forward
            (0, 0),      # Stop
            (-50, -50),  # Backward
            (0, 0),      # Stop
            (-30, 30),   # Turn left
            (0, 0),      # Stop
            (30, -30),   # Turn right
            (0, 0),      # Stop
        ]
        
        # Get current step in sequence
        current_cmd = test_sequences[self.test_sequence_step]
        self.test_sequence_step = (self.test_sequence_step + 1) % len(test_sequences)
        
        # Log what we're doing
        left_pwm, right_pwm = current_cmd
        if left_pwm == 0 and right_pwm == 0:
            self.get_logger().info(">>> TEST: STOPPING")
        elif left_pwm > 0 and right_pwm > 0:
            self.get_logger().info(">>> TEST: MOVING FORWARD")
        elif left_pwm < 0 and right_pwm < 0:
            self.get_logger().info(">>> TEST: MOVING BACKWARD")
        elif left_pwm < 0 and right_pwm > 0:
            self.get_logger().info(">>> TEST: TURNING LEFT") 
        elif left_pwm > 0 and right_pwm < 0:
            self.get_logger().info(">>> TEST: TURNING RIGHT")
            
        # Apply motor commands
        self.get_logger().info(f">>> TEST: Setting motors to Left={left_pwm}, Right={right_pwm}")
        self.direct_motor_control(left_pwm, right_pwm)

    def check_cmd_vel(self):
        """Periodically check cmd_vel activity"""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        
        # Only log if no messages have been received for a while
        if self.cmd_vel_count == 0 or elapsed > 30.0:
            self.get_logger().info(f">>> No cmd_vel messages received in {elapsed:.1f} seconds")
            
        # Log a simple summary once per minute
        if (self.get_clock().now().nanoseconds / 1e9) % 60 < 15:
            self.get_logger().info(f">>> Total cmd_vel messages: {self.cmd_vel_count}")

    def direct_motor_control(self, left_pwm, right_pwm):
        """Direct motor control with PWM values"""
        try:
            if self.is_shutting_down:
                return

            # Re-init if something's gone wrong
            if not (self.motor_hat and self.left_motor and self.right_motor):
                self.get_logger().error("Motors unavailable—reinitializing")
                if not self.setup_motors():
                    return

            # Apply the motor calibration factor to left motor
            if left_pwm != 0:
                # Log the factor being applied
                self.get_logger().info(f"Applying factor {self.left_motor_factor:.4f} to left motor")
                left_pwm = int(left_pwm * self.left_motor_factor)
                # Keep within PWM bounds
                if left_pwm > 0:
                    left_pwm = min(left_pwm, self.max_pwm)
                else:
                    left_pwm = max(left_pwm, -self.max_pwm)

            # LEFT MOTOR
            if left_pwm > 0:
                speed = min(left_pwm, self.max_pwm)
                self.left_motor.setSpeed(speed)
                self.left_motor.run(Adafruit_MotorHAT.FORWARD)
            elif left_pwm < 0:
                speed = min(-left_pwm, self.max_pwm)
                self.left_motor.setSpeed(speed)
                self.left_motor.run(Adafruit_MotorHAT.BACKWARD)
            else:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)

            # RIGHT MOTOR
            if right_pwm > 0:
                speed = min(right_pwm, self.max_pwm)
                self.right_motor.setSpeed(speed)
                self.right_motor.run(Adafruit_MotorHAT.FORWARD)
            elif right_pwm < 0:
                speed = min(-right_pwm, self.max_pwm)
                self.right_motor.setSpeed(speed)
                self.right_motor.run(Adafruit_MotorHAT.BACKWARD)
            else:
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)

        except Exception as e:
            self.get_logger().error(f"Direct motor control failed: {e}")
            self.setup_motors()

    def publish_transforms(self):
        """Periodically publish transforms regardless of encoder updates"""
        # Only publish transforms if we've processed at least one odometry update
        # or if we're in a mode where odometry is disabled
        if not self.odometry_processed and not self.disable_arduino:
            # Don't publish transforms until we have some valid odometry data
            return
            
        current_time = self.get_clock().now()
            
        # Create and publish odom → base_link transform
        odom_transform = TransformStamped()
        odom_transform.header.stamp = current_time.to_msg()
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        
        # Set translation
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0
        
        # Set rotation (quaternion from yaw)
        q = euler2quat(0, 0, self.theta)
        odom_transform.transform.rotation.w = q[0]
        odom_transform.transform.rotation.x = q[1]
        odom_transform.transform.rotation.y = q[2]
        odom_transform.transform.rotation.z = q[3]
        
        # Create and publish base_link → base_footprint transform
        footprint_transform = TransformStamped()
        footprint_transform.header.stamp = current_time.to_msg()
        footprint_transform.header.frame_id = "base_link"
        footprint_transform.child_frame_id = "base_footprint"
        
        # Set translation - based on wheel_radius + caster_height from URDF
        footprint_transform.transform.translation.x = 0.0
        footprint_transform.transform.translation.y = 0.0
        footprint_transform.transform.translation.z = -(self.wheel_radius + self.caster_height)  # Negative because going from base_link DOWN to base_footprint
        
        # Set rotation (identity quaternion - no rotation)
        footprint_transform.transform.rotation.w = 1.0
        footprint_transform.transform.rotation.x = 0.0
        footprint_transform.transform.rotation.y = 0.0
        footprint_transform.transform.rotation.z = 0.0
        
        # Broadcast both transforms
        self.tf_broadcaster.sendTransform([odom_transform, footprint_transform])
        
        # Log odometry occasionally but not too frequently
        time_since_last = (current_time - self.last_encoder_display_time).nanoseconds / 1e9
        if time_since_last >= 15.0:  # Reduced frequency to avoid log spam
            self.get_logger().info(f"Publishing TF - Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
            self.last_encoder_display_time = current_time

    def update_motor_pid(self, prev_left, prev_right):
        """Update PID controller to balance motor speeds"""
        current_time = time.time()
        dt = current_time - self.last_pid_time
        
        # Only update if we have movement and sufficient time has passed
        if dt > 0.1 and (prev_left != self.encoder1_count or prev_right != self.encoder2_count):
            # Calculate speeds
            left_delta = self.encoder1_count - prev_left
            right_delta = self.encoder2_count - prev_right
            
            # Skip if both wheels are stopped
            if left_delta == 0 and right_delta == 0:
                return
                
            # Calculate the difference/error between wheels (normalized)
            # Positive error means right wheel is faster than left
            if left_delta != 0 or right_delta != 0:
                # Determine current direction
                current_direction = None
                if left_delta > 0 and right_delta > 0:
                    current_direction = "forward"
                elif left_delta < 0 and right_delta < 0:
                    current_direction = "backward"
                
                # Reset PID if direction changed
                if current_direction and current_direction != self.prev_direction:
                    self.speed_pid.reset()
                    self.get_logger().info(f"Direction changed to {current_direction} - PID reset")
                
                # Store direction for next time
                self.prev_direction = current_direction
                
                # Calculate speed ratio difference
                if left_delta > 0 and right_delta > 0:
                    # Both wheels moving forward - error is the difference in speed ratio
                    error = (right_delta / max(1, left_delta)) - 1.0
                    # Log every 5 seconds or for significant errors
                    now = self.get_clock().now()
                    time_since_last = (now - self.last_encoder_display_time).nanoseconds / 1e9
                    if time_since_last >= 3.0:
                        self.get_logger().info(f"Speed difference detected: L: {left_delta}, R: {right_delta}, Error: {error:.4f}")
                elif left_delta < 0 and right_delta < 0:
                    # Both wheels moving backward - error is the difference in speed ratio
                    error = (right_delta / min(-1, left_delta)) - 1.0
                else:
                    # Mixed direction or one wheel stopped - don't adjust
                    error = 0
                    
                # Only adjust if error is significant - lowered threshold 
                if abs(error) > 0.005:
                    # Compute PID output
                    adjustment = self.speed_pid.compute(error)
                    
                    # Cap the adjustment to reasonable values - increased range
                    adjustment = max(-0.3, min(0.3, adjustment))
                    
                    # Always log adjustments - more verbose
                    self.get_logger().info(f"PID Adjustment: {adjustment:.4f} (Error: {error:.4f}, L: {left_delta}, R: {right_delta})")
                    
                    # Store the adjustment factor for use in direct_motor_control
                    self.left_motor_factor = 1.015 + adjustment
                    
            # Update timestamp
            self.last_pid_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WaffleBotNode()
    
    # Setup signal handlers for graceful shutdown
    import signal
    import sys
    
    def signal_handler(sig, frame):
        node.stop_motors()  # Explicitly stop motors
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register SIGINT handler (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This is a fallback - the signal handler should catch this first
        node.stop_motors()
    finally:
        # Make absolutely sure motors are stopped
        node.stop_motors()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
