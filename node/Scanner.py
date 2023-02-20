import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):

    print(msg.ranges)


if __name__ == '__main__':

    rospy.init_node('lidar_subscriber')

    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.spin()