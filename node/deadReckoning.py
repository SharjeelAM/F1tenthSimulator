#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion

class DeadReckoning:
    def __init__(self):
        rospy.init_node('dead_reckoning', anonymous=True)

        self.speed = 2.0 # constant speed for car to move
        self.angle = 0.0 # steering angle
        self.x = 0.0 # initial x position
        self.y = 0.0 # initial y position
        self.yaw = 0.0 # initial yaw angle

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def laser_scan_callback(self, msg):
        # check if there's any obstacle within 1 meter in front
        if min(msg.ranges[len(msg.ranges) // 2 - 5:len(msg.ranges) // 2 + 5]) < 1.0:
            # stop the car if there's an obstacle
            self.speed = 0.0
        else:
            # move the car forward otherwise
            self.speed = 2.0

    def odometry_callback(self, msg):
        # update the x, y, and yaw using dead reckoning
        dt = (msg.header.stamp - rospy.Time.now()).to_sec()
        self.x += self.speed * dt * math.cos(self.yaw)
        self.y += self.speed * dt * math.sin(self.yaw)
        self.yaw += self.angle * dt

        # create and publish the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'drive'
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.angle
        self.pub.publish(drive_msg)

if __name__ == '__main__':
    node = DeadReckoning()
    rospy.spin()
