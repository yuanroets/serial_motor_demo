import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock



class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')


        # Setup parameters

        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")


        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")


        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value


        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")

        # Motor calibration parameters to correct for wheel bias
        self.declare_parameter('motor_1_scaler', value=1.0)  # Left motor - testing navigation with original values
        self.declare_parameter('motor_2_scaler', value=1.0)  # Right motor correction
        self.motor_1_scaler = self.get_parameter('motor_1_scaler').value
        self.motor_2_scaler = self.get_parameter('motor_2_scaler').value
        print(f"Motor scalers - Motor 1 (left): {self.motor_1_scaler}, Motor 2 (right): {self.motor_2_scaler}")



        # Setup topics & services

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        

        # Member Variables

        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()


        # Open serial comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")
        

        


    # Raw serial commands
    
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        cmd = f"o {int(mot_1_pwm)} {int(mot_2_pwm)}"
        print(f"DRIVER: Sending PWM command: {cmd}")
        self.send_command(cmd)

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        cmd = f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}"
        print(f"DRIVER: Sending feedback command: {cmd}")
        self.send_command(cmd)

    def send_encoder_read_command(self):
        resp = self.send_command(f"e")
        if resp:
            return [int(raw_enc) for raw_enc in resp.split()]
        return []


    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        print(f"DRIVER: Received motor command - PWM Mode: {motor_command.is_pwm}, Motor 1: {motor_command.mot_1_req_rad_sec}, Motor 2: {motor_command.mot_2_req_rad_sec}")
        
        if (motor_command.is_pwm):
            print(f"DRIVER: Sending PWM command to motors")
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            base_scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            
            # Apply individual motor scalers to correct for wheel bias
            # Get current parameter values (allows runtime tuning)
            current_motor_1_scaler = self.get_parameter('motor_1_scaler').value
            current_motor_2_scaler = self.get_parameter('motor_2_scaler').value
            
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * base_scaler * current_motor_1_scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * base_scaler * current_motor_2_scaler
            
            print(f"DRIVER: Sending feedback command - Motor 1 counts: {mot1_ct_per_loop}, Motor 2 counts: {mot2_ct_per_loop}")
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):

            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
            self.m1_spd = m1_diff*rads_per_ct/time_diff
            self.m2_spd = m2_diff*rads_per_ct/time_diff

            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)



    # Utility functions

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            print(f"DRIVER: About to send serial command: {cmd_string.strip()}")
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            print(f"DRIVER: Serial response: {value}")
            return value
        except Exception as e:
            print(f"DRIVER: Serial communication error: {e}")
            return ''
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()



def main(args=None):
    
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    # ROS Jazzy compatible approach - create_rate is deprecated
    # Use a timer-based approach instead of the polling loop
    def check_encoders_callback():
        motor_driver.check_encoders()
    
    # Use 20Hz rate for better SLAM odometry (was 2Hz = 0.5s, now 20Hz = 0.05s)
    timer = motor_driver.create_timer(0.05, check_encoders_callback)
    
    try:
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        print("Shutting down motor driver...")
    finally:
        motor_driver.close_conn()
        motor_driver.destroy_node()
        rclpy.shutdown()


