import rospy
from std_msgs.msg import Float32

class SimpleNode:
    def __init__(self):
        rospy.init_node('simple_node')

        self.pub = rospy.Publisher('/nav_msgs/Odometry', Float32, queue_size=10)

        self.rate = rospy.Rate(10)


    def run(self):
        while not rospy.is_shutdown():
            speed = 500
            self.pub.publish(speed)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SimpleNode()
        node.run()
    except rospy.ROSInterruptionException:
        pass