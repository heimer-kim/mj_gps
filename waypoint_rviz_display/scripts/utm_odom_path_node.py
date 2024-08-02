#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class PathNode:
    def __init__(self):
        self.path = Path()
        self.odom_sub = rospy.Subscriber('/utm_odom', Odometry, self.odom_cb)
        self.reset_odom_sub = rospy.Subscriber('/reset_path', Bool, self.reset_path_cb)
        self.path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)

    def odom_cb(self, data):
        self.path.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        rospy.loginfo("Path updated")

    def reset_path_cb(self, data):
        self.path.poses = []
        rospy.loginfo("Path reset")

if __name__ == '__main__':
    rospy.init_node('path_node')
    node = PathNode()
    rospy.spin()

'''
#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

path = Path()

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)
    
    
def reset_path_cb(data):
    
    path.poses=[]
    print("REST")
	    

rospy.init_node('path_node')

odom_sub = rospy.Subscriber('/utm_odom', Odometry, odom_cb)
reset_odom_sub = rospy.Subscriber('/reset_path', Bool, reset_path_cb)
path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)

if __name__ == '__main__':
    rospy.spin()
'''
