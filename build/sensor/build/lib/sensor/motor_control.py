import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class chasis_controller(Node):
    def __init__(self):
        super().__init__('Controller_node')

        self.US_left = self.create_subscription(Float32,'/left_sensor_measurement',self.left_callback,10)
        self.US_right = self.create_subscription(Float32,'/right_sensor_measurement',self.right_callback,10)
        self.US_middle = self.create_subscription(Float32,'/middle_sensor_measurement',self.middle_callback,10)



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
