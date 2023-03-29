import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollowerNode:
    def __init__(self):

        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.goal_distance = 0.5
        self.max_speed = 2.0
        self.max_angle = 1.0

    def scan_callback(self, msg):
        left_wall_distance = sum(msg.ranges[405:540]) / 60
        right_wall_distance = sum(msg.ranges[0:135]) / 60

        angle = (right_wall_distance - left_wall_distance) / 2.0

        angle = max(-self.max_angle, min(angle, self.max_angle))

        speed = min(self.max_speed, left_wall_distance - self.goal_distance)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollowerNode()
    node.run()