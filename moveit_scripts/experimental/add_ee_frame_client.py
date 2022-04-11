#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from moveit_scripts.srv import *


if __name__ == '__main__':
    rospy.init_node('add_ee_frame_client', anonymous=True)
    rospy.wait_for_service('add_ee_frame')
    add_ee_frame = rospy.ServiceProxy("add_ee_frame", AddNewEeFrame)
    new_frame = geometry_msgs.msg.Transform()
    new_frame.translation.x = 1.0
    new_frame.translation.y = 1.0
    new_frame.translation.z = 1.0
    new_frame.rotation.x = 0.0
    new_frame.rotation.y = 0.0
    new_frame.rotation.z = 0.0
    new_frame.rotation.w = 1.0


    resp = get_goal_ee(new_frame)

    if(resp.success.data == True):
        print("Received response")
