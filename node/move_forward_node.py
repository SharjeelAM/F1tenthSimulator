#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

    
def run():
        while not rospy.is_shutdown():
                drive_cmd = AckermannDriveStamped()
                drive_cmd.drive.speed = 5
                drive_pub.publish(drive_cmd)
            


if __name__ == "__main__":
        try:
                rospy.init_node("move_forward_node")
                drive_pub = rospy.Publisher("/f1tenth/ackermann_cmd", AckermannDriveStamped, queue_size=10)
                rate = rospy.Rate(500)
                run()
        except rospy.ROSInterruptException:
                pass