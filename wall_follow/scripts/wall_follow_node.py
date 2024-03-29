#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import time

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # TODO: set PID gains
        self.kp = 1.0
        self.kd = 0.01
        self.ki = 0.05

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.current_time = time.time()
        self.lookahead_distance = 1.0 # 

        # initial 
        self.angle_min = 0.0
        self.angle_increment = 0.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        # TODO: implement
        index = int(np.floor((angle - self.angle_min)/self.angle_increment))
        return range_data[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        # TODO:implement
        angle_a = np.pi/4 # 45 degree
        angle_b = np.pi/2 # 90 degree
        theta = angle_b - angle_a
        
        while (not np.isfinite(self.get_range(range_data, angle_a)) or (angle_a == angle_b)):
            angle_a += self.angle_increment
        while (not np.isfinite(self.get_range(range_data, angle_b)) or (angle_a == angle_b)):
            angle_b += self.angle_increment

        a = self.get_range(range_data, angle_a)
        b = self.get_range(range_data, angle_b)
        
        alpha = np.arctan2((a * np.cos(theta) - b), (a * np.sin(theta)))
        current_distance = b * np.cos(alpha)
        new_distance = current_distance + (self.lookahead_distance * np.sin(alpha))
        error_distance = dist - new_distance 

        return error_distance

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0

        previous_time = self.current_time
        self.current_time = time.time()
        dt = self.current_time - previous_time

        # TODO: Use kp, ki & kd to implement a PID controller

        self.integral = self.prev_error * dt
        derivative =  (error - self.prev_error) / dt
        desired_angle = -((self.kp * error) + (self.kd * derivative) + (self.ki * self.integral))

        if abs(error) < 0.1:
            angle = 0.0

        if 0 < desired_angle < np.pi/18: # 0 degree < angle < 10 degree
            velocity = 1.5
        elif np.pi/18 <= desired_angle < np.pi/9: # 10 degree <= angle < 20 degree
            velocity = 1.0
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = desired_angle

        self.publisher_drive.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        error = self.get_error(msg.ranges, 1.0) # TODO: replace with error calculated by get_error(error, dist)
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()