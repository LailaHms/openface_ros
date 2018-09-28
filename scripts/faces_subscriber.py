#!/usr/bin/env python
import rospy
from openface_ros.msg import Faces
from openface_ros.msg import Face
from std_msgs.msg import Int32


def faces_count_callback(msg):
    rospy.loginfo("Number of faces detected : %d", len(msg.faces)) 

if __name__=='__main__':
    rospy.init_node('faces_subscriber')
    
    sub=rospy.Subscriber('faces', Faces, faces_count_callback)
    rospy.spin()
