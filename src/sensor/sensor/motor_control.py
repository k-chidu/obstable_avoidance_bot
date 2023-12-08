import rclpy
from rclpy.node import Node
import time
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
        
        self.left = 0
        self.right = 0
        self.middle = 0
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
        print("left =", self.left)

    def right_callback(self, msg: Float32):
        self.right = msg.data
        print("right =", self.right)

    def middle_callback(self, msg: Float32):
        self.middle = msg.data
        print("middle =", self.middle)

        # Your main logic goes here
        if self.middle <= 15.0:
            # Reduce PWM to stop
            for duty_cycle in range(10, 0, -1):  # Start from 20% and reduce to 0%
                self.pwm_left_fwd.ChangeDutyCycle(duty_cycle)
                self.pwm_right_fwd.ChangeDutyCycle(duty_cycle)
                time.sleep(0.01)
            # Stop PWM
            self.pwm_left_fwd.stop()
            self.pwm_right_fwd.stop()
        else:
            print("insider")
            # Provide constant PWM of 20%
            self.pwm_left_fwd.start(10)
            self.pwm_right_fwd.start(10)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ChassisController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()  # Cleanup GPIO when exiting

if __name__ == '__main__':
    main()
