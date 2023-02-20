import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class MoveForward:
    def __init__(self):

        self.nh = rospy.init_node("move_forward_node")

        self.drive_pub = rospy.Publisher("/f1tenth/ackermann_cmd", AckermannDriveStamped, queue_size=10)

        self.rate = rospy.Rate(10)

    
    def run(self):

        while not rospy.is_shutdown():
            drive_cmd = AckermannDriveStamped()
            drive_cmd.header.stamp = rospy.Time.now()
            drive_cmd.header.frame_id = "base_link"
            drive_cmd.drive.speed = 5
            drive_cmd.drive.steering_angle = 0.0
            drive_cmd.drive.acceleration = 5
            self.drive_pub.publish(drive_cmd)
            self.rate.sleep()


if __name__ == "__main__":

    move_forward = MoveForward()

    move_forward.run()
