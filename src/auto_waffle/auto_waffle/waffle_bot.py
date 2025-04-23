import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from transforms3d.euler import euler2quat
import serial
import serial.tools.list_ports
import math
import time
import os
import subprocess
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class WaffleBotNode(Node):
    def __init__(self):
        super().__init__('waffle_bot_node')
        self.get_logger().info(">>> Initializing WaffleBotNode")

        # Motor control parameters
        self.min_pwm = 60  # Minimum PWM to overcome motor inertia
        self.max_pwm = 255  # Maximum PWM value
        self.pwm_scale = 1.0  # Scale factor for PWM values
        self.max_linear_speed = 0.5  # Max linear speed from teleop (m/s)
        self.max_angular_speed = 1.0  # Max angular speed from teleop (rad/s)
        
        # Motor configuration
        self.m1_power_boost = 1.5  # Boost M1 power by this factor
        self.swap_motors = False  # Set to True to swap M1 and M2 if wired incorrectly
        
        # Test mode flag
        self.test_mode = False  # Set to False to disable automatic motor testing
        self.test_sequence_step = 0
        
        # Add cmd_vel tracking variables
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_count = 0
        
        # Arduino and encoder variables
        self.disable_arduino = False  # Set to True to disable Arduino connection attempts
        self.arduino_baudrate = 9600  # Arduino baud rate - try different values if not working
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
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
        # Add timers
        self.last_time = self.get_clock().now()
        if not self.disable_arduino:  # Create Arduino timers if Arduino is enabled
            # Create timer to check for Arduino data at 10Hz
            self.encoder_timer = self.create_timer(0.1, self.update_odometry)
            # Create timer to check Arduino connection at 1Hz
            self.connection_timer = self.create_timer(1.0, self.check_arduino_connection)
        
        # Add a motor check timer
        self.create_timer(1.0, self.check_motor_hat)
        
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
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = 0.03  # meters
        self.wheel_base = 0.18    # meters
        self.ticks_per_rev = 360
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
        # Common Arduino identifiers
        arduino_ids = [
            'Arduino',
            'USB2.0-Serial',
            'ACM',
            'USB Serial'
        ]
        
        # Use the configured baud rate
        arduino_baud_rate = self.arduino_baudrate
        self.get_logger().info(f"Trying to connect to Arduino at {arduino_baud_rate} baud")
        
        ports = list(serial.tools.list_ports.comports())
        self.get_logger().info(f"Available ports: {[p.device for p in ports]}")
        
        for port in ports:
            # Check if any of the Arduino identifiers are in the port description
            if any(id in port.description for id in arduino_ids) or port.device in ['/dev/ttyACM0', '/dev/ttyUSB0']:
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
        common_ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']
        for port in common_ports:
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
            # EMERGENCY OVERRIDE - When any cmd_vel is received, send a fixed motor command
            # to verify that motors can respond to cmd_vel
            
            # Check if there's any significant movement requested
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                # Calculate left and right PWM values
                left_pwm = 0
                right_pwm = 0
                
                # If linear.x is positive, move forward
                if msg.linear.x > 0:
                    left_pwm = 70
                    right_pwm = 50
                # If linear.x is negative, move backward
                elif msg.linear.x < 0:
                    left_pwm = -70
                    right_pwm = -50
                # If angular.z is positive, turn left
                elif msg.angular.z > 0:
                    left_pwm = -70
                    right_pwm = 50
                # If angular.z is negative, turn right
                elif msg.angular.z < 0:
                    left_pwm = 70
                    right_pwm = -50
                else:
                    left_pwm = 0
                    right_pwm = 0
                
                # Apply M1 power boost
                left_pwm = int(left_pwm * self.m1_power_boost) if left_pwm != 0 else 0
                
                # Swap motors if configured
                if self.swap_motors:
                    left_pwm, right_pwm = right_pwm, left_pwm
                    
                self.direct_motor_control(left_pwm, right_pwm)
                return
            
            # ORIGINAL CODE - Only reached if the emergency override doesn't activate
            # SIMPLIFIED LOGIC FOR TESTING
            # Use direct mapping from linear.x to forward/backward
            # and angular.z to turning
            # Scale from -1.0 to 1.0 to -100 to 100
            
            # Linear velocity controls forward/backward
            linear_pwm = int(msg.linear.x * 100)  # Scale to -100 to 100
            
            # Angular velocity controls turning
            turn_pwm = int(msg.angular.z * 50)  # Scale to -50 to 50
            
            # Combine to get left and right wheel PWM values
            left_pwm = linear_pwm - turn_pwm
            right_pwm = linear_pwm + turn_pwm
            
            # Ensure values are within range
            left_pwm = max(min(left_pwm, 100), -100)
            right_pwm = max(min(right_pwm, 100), -100)
            
            self.get_logger().info(f">>> SIMPLIFIED CONVERSION: linear_pwm={linear_pwm}, turn_pwm={turn_pwm}")
            self.get_logger().info(f">>> SENDING TO MOTORS - Left: {left_pwm}, Right: {right_pwm}")
            
            # Special case for testing: force a minimum value if small command received
            if 0 < abs(linear_pwm) < 20 or 0 < abs(turn_pwm) < 20:
                self.get_logger().info(">>> Small command detected - boosting signal for testing")
                if linear_pwm > 0:
                    linear_pwm = 30
                elif linear_pwm < 0:
                    linear_pwm = -30
                
                if turn_pwm > 0:
                    turn_pwm = 20
                elif turn_pwm < 0:
                    turn_pwm = -20
                    
                left_pwm = linear_pwm - turn_pwm
                right_pwm = linear_pwm + turn_pwm
                left_pwm = max(min(left_pwm, 100), -100)
                right_pwm = max(min(right_pwm, 100), -100)
                
                self.get_logger().info(f">>> BOOSTED SIGNALS - Left: {left_pwm}, Right: {right_pwm}")
            
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
                                for part in parts:
                                    if "Motor 1:" in part:
                                        motor1_str = part.replace("Motor 1:", "").strip()
                                        try:
                                            self.encoder1_count = int(motor1_str)
                                        except ValueError:
                                            pass
                                
                                # Extract Motor 2 count (should be in part at index 3)
                                for part in parts:
                                    if "Motor 2:" in part:
                                        motor2_str = part.replace("Motor 2:", "").strip()
                                        try:
                                            self.encoder2_count = int(motor2_str)
                                        except ValueError:
                                            pass
                                
                                # Display encoder counts at regular intervals
                                self.display_encoder_data()
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
            # Don't attempt to control motors if shutting down
            if self.is_shutting_down:
                self.get_logger().info(">>> Node is shutting down - ignoring motor commands")
                return
                
            # Verify Motor HAT connection before sending commands
            if not self.motor_hat or not self.left_motor or not self.right_motor:
                self.get_logger().error(">>> Motors not available! Reinitializing...")
                if not self.setup_motors():
                    return
            
            # Process left motor (M3)
            if left_pwm > 0:
                # Use a higher initial PWM for left motor to overcome inertia
                initial_pwm = 150  # Higher value to give more initial power
                self.left_motor.setSpeed(initial_pwm)
                self.left_motor.run(Adafruit_MotorHAT.FORWARD)
                time.sleep(0.05)  # Short delay to let the motor overcome inertia
                
                # Then set to the normal speed
                self.left_motor.setSpeed(100)
                left_actual_pwm = 100
                left_direction = "FORWARD"
            elif left_pwm < 0:
                # Use a higher initial PWM for left motor to overcome inertia
                initial_pwm = 150  # Higher value to give more initial power
                self.left_motor.setSpeed(initial_pwm)
                self.left_motor.run(Adafruit_MotorHAT.BACKWARD)
                time.sleep(0.05)  # Short delay to let the motor overcome inertia
                
                # Then set to the normal speed
                self.left_motor.setSpeed(100)
                left_actual_pwm = 100
                left_direction = "BACKWARD"
            else:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                left_actual_pwm = 0
                left_direction = "RELEASE"
            
            # Process right motor (M2)
            if right_pwm > 0:
                self.right_motor.setSpeed(100)
                self.right_motor.run(Adafruit_MotorHAT.FORWARD)
                right_actual_pwm = 100
                right_direction = "FORWARD"
            elif right_pwm < 0:
                self.right_motor.setSpeed(100)
                self.right_motor.run(Adafruit_MotorHAT.BACKWARD)
                right_actual_pwm = 100
                right_direction = "BACKWARD"
            else:
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
                right_actual_pwm = 0
                right_direction = "RELEASE"
            
        except Exception as e:
            self.get_logger().error(f">>> Direct motor control failed: {str(e)}")
            # Try to reinitialize motors
            self.setup_motors()

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
