import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
import numpy as np
import math
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
    
    def inv_kine(self, vel,w):
        matrix = np.matrix([[1, 0], [1, 1]])
        V_b = np.matrix([[vel],[w]])
        V = np.matmul(matrix,V_b)
        print(V)
        try:
            if(V[0]>0):
                self.pwm_left_fwd.start(abs(V[0]))
            else:
                self.pwm_left_bwd.start(abs(V[0]))

            if(V[1]>0):
                self.pwm_right_fwd.start(abs(V[1]))
            else:
                self.pwm_right_bwd.start(abs(V[1]))

    
        except:
            if(V[0]>0):
                self.pwm_left_fwd.ChangeDutyCycle(abs(V[0]))
            else:
                self.pwm_left_bwd.ChangeDutyCycle(abs(V[0]))
  
            if(V[1]>0):
                self.pwm_right_fwd.ChangeDutyCycle(abs(V[1]))
            else:
                self.pwm_right_bwd.ChangeDutyCycle(abs(V[1]))

            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisController()
    try:
        while rclpy.ok():
            # Your main logic goes here
            if node.middle <= 30.0:
                # Reduce PWM to stop
                """
                for duty_cycle in range(10, 0, -1):  # Start from 20% and reduce to 0%
                    node.pwm_left_fwd.ChangeDutyCycle(duty_cycle)
                    node.pwm_right_fwd.ChangeDutyCycle(duty_cycle)
                    time.sleep(0.01)
                
                
                    
                # Stop PWM
                node.pwm_left_fwd.stop()
                node.pwm_right_fwd.stop()
                """

                if node.right>=15.0:
                    if node.middle>15.0:
                        vel = 10
                    else:
                        vel = 10*(1-math.exp(-node.middle/50))
                        print("vel=",vel)
                    w = (45)*0.8
                    node.inv_kine(vel,w)

                if node.left >=15.0:
                    if node.middle>15.0:
                        vel = 10
                    else:
                        vel = 10*(1 - math.exp(-node.middle/50))
                    w= (-45)*-0.8
                    node.inv_kine(vel,w)
            else:
                # Provide constant PWM of 20%
                #node.pwm_left_fwd.start(10)
                #node.pwm_right_fwd.start(10)
                node.inv_kine(10,0)
                print("moving straight")
                time.sleep(0.01)
            
            rclpy.spin_once(node, timeout_sec=0.1)  # Adjust the timeout as needed
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Cleanup GPIO when exiting

if __name__ == '__main__':
    main()
