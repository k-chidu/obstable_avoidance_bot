#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('left_sensor_node')
        self.publisher_ = self.create_publisher(Float32, 'left_sensor_measurement', 1)
        self.timer = self.create_timer(0.1, self.publish_distance)

    def measure_distance(self):
        pulse_start_time = 0
        pulse_end_time = 0

        # Trigger pulse
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # Measure time for echo
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start_time = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end_time = time.time()

        # Calculate distance in centimeters
        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * 17150

        return distance

    def publish_distance(self):
        dist = self.measure_distance()

        msg = Float32()
        msg.data = round(dist,2)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Distance: {dist}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    TRIG_PIN = 23
    ECHO_PIN = 24

    # Setup GPIO pins
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    main()
