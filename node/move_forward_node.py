#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

    
def run():
        print("This shit running")
        while not rospy.is_shutdown():
                drive_cmd = AckermannDriveStamped()
                drive_cmd.drive.speed = 2
                drive_pub.publish(drive_cmd)
            


if __name__ == "__main__":
        try:
                rospy.init_node("move_forward_node")
                drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
                run()
        except rospy.ROSInterruptException:
                pass












