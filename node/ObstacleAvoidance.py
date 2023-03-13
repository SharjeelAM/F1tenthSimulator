#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("obstacle_avoidance")
        self.sub_lidar = rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.pub_drive = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.speed = 1.0
        self.angle = 0.0
        self.goal_distance = 1000000000000000000000000000000000000000000000000.0

    def laser_scan_callback(self, msg):
        obstacle_detected = False
        min_range = 0.25
        angle_range = 540

        for i in range(-angle_range, angle_range):
            if msg.ranges[i] < min_range:
                obstacle_detected = True
                break
        
        if obstacle_detected:
            print("Obstacle Detected")
            if msg.ranges[0] > self.goal_distance:
                print("Test1")
                self.angle = 0.0
            else:
                print("Test2")
                self.angle = 50000 if self.angle < 540 else -50000
            self.speed = 1.0
        else:
            self.angle = 0.0
            self.speed = 1.0
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.angle
        self.pub_drive.publish(drive_msg)



if __name__ == "__main__":
    avoidance = ObstacleAvoidance()
    rospy.spin()