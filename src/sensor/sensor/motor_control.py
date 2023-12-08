import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from std_msgs.msg import Float32

class chasis_controller(Node):
    def __init__(self):
        super().__init__('Controller_node')

        self.US_left = self.create_subscription(Float32,'/left_sensor_measurement',self.left_callback,10)
        self.US_right = self.create_subscription(Float32,'/right_sensor_measurement',self.right_callback,10)
        self.US_middle = self.create_subscription(Float32,'/middle_sensor_measurement',self.middle_callback,10)
        GPIO.setmode(GPIO.BCM)
        self.left_fwd = 18 #D
        self.right_fwd = 12 #A
        self.left_bwd =  13 #B
        self.left_fwd =  19 #C


        try:
            GPIO.setup(self.left_fwd, GPIO.OUT)
            GPIO.setup(self.right_fwd, GPIO.OUT)
            GPIO.setup(self.left_bwd, GPIO.OUT)
            GPIO.setup(self.left_bwd, GPIO.OUT)

            pwm_a = GPIO.pwm(self.left_fwd ,1000)
            pwm_a = GPIO.pwm(self.right_fwd ,1000)
            pwm_a = GPIO.pwm(self.left_bwd ,1000)
            pwm_a = GPIO.pwm(self.left_bwd ,1000)


    def left_callback(self, msg : Float32):
        
        self.vel_left = msg.data
        print("left = ",self.vel_left)

    def right_callback(self, msg : Float32):
        
        self.vel_right = msg.data
        print("right = ",self.vel_right)

    def middle_callback(self, msg : Float32):
        

        self.vel_middle = msg.data
        print("middle = ",self.vel_middle)

def main(args=None):
    rclpy.init(args=args)
    node = chasis_controller()
    rclpy.spin(node)

if __name__ =='__main__':
    main()
