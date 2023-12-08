import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from std_msgs.msg import Float32

class ChassisController(Node):
    def __init__(self):
        super().__init__('Controller_node')

        self.US_left = self.create_subscription(Float32, '/left_sensor_measurement', self.left_callback, 10)
        self.US_right = self.create_subscription(Float32, '/right_sensor_measurement', self.right_callback, 10)
        self.US_middle = self.create_subscription(Float32, '/middle_sensor_measurement', self.middle_callback, 10)

        GPIO.setmode(GPIO.BCM)
        self.left_fwd = 18  # D
        self.right_fwd = 12  # A
        self.left_bwd = 13  # B
        self.right_bwd = 19  # C

        try:
            GPIO.setup(self.left_fwd, GPIO.OUT)
            GPIO.setup(self.right_fwd, GPIO.OUT)
            GPIO.setup(self.left_bwd, GPIO.OUT)
            GPIO.setup(self.right_bwd, GPIO.OUT)

            self.pwm_left_fwd = GPIO.PWM(self.left_fwd, 1000)
            self.pwm_right_fwd = GPIO.PWM(self.right_fwd, 1000)
            self.pwm_left_bwd = GPIO.PWM(self.left_bwd, 1000)
            self.pwm_right_bwd = GPIO.PWM(self.right_bwd, 1000)
        except:
            pass

    def left_callback(self, msg: Float32):
        self.left = msg.data
        #print("left =", self.vel_left)

    def right_callback(self, msg: Float32):
        self.right = msg.data
        #print("right =", self.vel_right)

    def middle_callback(self, msg: Float32):
        self.middle = msg.data
        #print("middle =", self.vel_middle)

def main(args=None):
    rclpy.init(args=args)
    node = ChassisController()
    try:
        while rclpy.ok():
            # Your main logic goes here
            if ChassisController.middle <= 10.0:
                # Reduce PWM to stop
                for duty_cycle in range(20, 0, -1):  # Start from 20% and reduce to 0%
                    node.pwm_left_fwd.ChangeDutyCycle(duty_cycle)
                    node.pwm_right_fwd.ChangeDutyCycle(duty_cycle)
                    rclpy.spin_once(node, timeout_sec=0.1)  # Allow time for the PWM change
                # Stop PWM
                node.pwm_left_fwd.stop()
                node.pwm_right_fwd.stop()
            else:
                # Provide constant PWM of 20%
                node.pwm_left_fwd.ChangeDutyCycle(20)
                node.pwm_right_fwd.ChangeDutyCycle(20)
                rclpy.spin_once(node, timeout_sec=0.1)  # Allow time for the PWM change
            
            rclpy.spin_once(node, timeout_sec=0.1)  # Adjust the timeout as needed
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Cleanup GPIO when exiting

if __name__ == '__main__':
    main()

