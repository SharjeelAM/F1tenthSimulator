#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SimplePlanner:
    def __init__(self):
        rospy.init_node('simple_planner', anonymous=True)

        self.speed = 1.0 # constant speed for car to move
        self.goal_distance = 1.0 # distance to maintain in front of the car
        self.angle = 0.0 # steering angle
        self.turn_angle = 0.9 # angle to turn when obstacle is detected
        self.turn_time = 3.0 # time to turn in seconds
        self.last_turn_time = 0.0 # last time the car turned
        self.turning = False # flag to indicate if the car is turning
        self.stop_distance = 0.5 # distance to stop the car when obstacle is detected

        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def laser_scan_callback(self, msg):
        # check if the car is turning
        if self.turning:
            # check if the turn time has elapsed
            if rospy.get_time() - self.last_turn_time > self.turn_time:
                self.turning = False # reset the turning flag
            else:
                # set the steering angle to turn angle
                self.angle = self.turn_angle
                # create and publish the drive message
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = 'drive'
                drive_msg.drive.speed = 0.0
                drive_msg.drive.steering_angle = self.angle
                self.pub.publish(drive_msg)
                return # exit the callback

        # check the distance in front of the car
        if msg.ranges[len(msg.ranges) // 2] < self.stop_distance:
            # stop the car and start turning
            self.turning = True
            self.last_turn_time = rospy.get_time()
            # create and publish the drive message
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = 'drive'
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = self.turn_angle
            self.pub.publish(drive_msg)
        else:
            # maintain the goal distance
            if msg.ranges[len(msg.ranges) // 2] > self.goal_distance:
                self.angle = 0.0
            else:
                self.angle = 0.5 if self.angle < 0.0 else -0.5
            # create and publish the drive message
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = 'drive'
            drive_msg.drive.speed = self.speed
            drive_msg.drive.steering_angle = self.angle
            self.pub.publish(drive_msg)

if __name__ == '__main__':
    node = SimplePlanner()
    rospy.spin()