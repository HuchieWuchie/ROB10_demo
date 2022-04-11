#!/usr/bin/env python   
import rospy
import geometry_msgs.msg
from moveit_scripts.srv import *

def handle_add_ee_frame(req):
    print("handle_add_ee_frame")
    res = AddNewEeFrameResponse()
    res.success.data = True
    return res

if __name__ == '__main__':
    rospy.init_node('add_ee_frame_server_node', anonymous=True)
    add_ee_frame_server = rospy.Service('add_ee_frame', AddNewEeFrame, handle_add_ee_frame)

    print("add new end-effector frame server is ready")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
