import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class CollisionAvoidance:
    def __init__(self):

        self.nh = rospy.init_node("collision_avoidance_node")

        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

        self.velocity_pub = rospy.Publisher("/f1tenth/velocity", Float32, queue_size=1000)

    def laser_scan_callback(self, msg):

        min_range = min(msg.ranges)

        if min_range < 0.5:

            self.velocity_pub.publish(Float32(0))

        else:

            self.velocity_pub.publish(Float32(0.5))


if __name__ == "__main__":

    collision_avoidance = CollisionAvoidance()

    rospy.spin()

    