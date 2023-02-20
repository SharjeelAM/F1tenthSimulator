import rospy
from geometry_msgs.msg import Twist

class StraightLine:
    def __init__(self):

        self.nh = rospy.init_node("straight_line_node")

        self.velocity_pub = rospy.Publisher("/f1tenth/velocity", Twist, queue_size=10)


    def run(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            vel_msg = Twist()
            vel_msg.linear.x = 0.5
            self.velocity_pub.publish(vel_msg)

            rate.sleep()

if __name__ == "__main__":

    straight_line = StraightLine()

    straight_line.run()