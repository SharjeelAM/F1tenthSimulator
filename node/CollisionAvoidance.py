import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


# def laser_scan_callback(msg):

#         min_range = min(msg.ranges)

#         if min_range<0.5:

#             drive_cmd = AckermannDriveStamped()
#             drive_cmd.drive.speed = 0
#             drive_pub.publish(drive_cmd)

#         else:
#             drive_cmd = AckermannDriveStamped()
#             drive_cmd.drive.speed = 2
#             drive_pub.publish(drive_cmd)


    


# if __name__ == "__main__":
#     try:
#         rospy.init_node("collision_avoidance_node")
#         rospy.Subscriber("/scan", LaserScan, laser_scan_callback)
#         drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass





class CollisionAvoidance:
    def __init__(self):

        self.nh = rospy.init_node("collision_avoidance_node")
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)



    def laser_scan_callback(self, msg):
        

        min_range = min(msg.ranges)

        if min_range < 0.5:
            drive_cmd = AckermannDriveStamped()
            drive_cmd.drive.speed = 0
            self.drive_pub.publish(drive_cmd)

        else:
            drive_cmd = AckermannDriveStamped()
            drive_cmd.drive.speed = 2
            self.drive_pub.publish(drive_cmd)


if __name__ == "__main__":

    collision_avoidance = CollisionAvoidance()
    rospy.spin()


    