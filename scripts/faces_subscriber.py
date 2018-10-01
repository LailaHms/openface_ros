#!/usr/bin/env python
import rospy
from openface_ros.msg import Faces
from openface_ros.msg import Face
from std_msgs.msg import Int32

pub = rospy.Publisher('/faces_count', Int32, queue_size=10)
n_faces = 0

def faces_handler():
    rospy.init_node('faces_subscriber', anonymous=True)
    rospy.Subscriber('faces', Faces, faces_count_callback)
    rospy.spin()

def faces_count_callback(msg):
    n_faces = len(msg.faces)
    rospy.loginfo("Number of faces detected : %d", n_faces)
    pub.publish(n_faces)

if __name__=='__main__':
    try:
        faces_handler()
    except rospy.ROSInterruptException:
        pass
