#!/usr/bin/env python3
import rospy
import std_msgs.msg
import geometry_msgs.msg
from rob10.srv import *


if __name__ == '__main__':
    rospy.init_node('scan_processing_client', anonymous=True)
    rospy.wait_for_service('requestReceiverPose')
    get_receiver_pose = rospy.ServiceProxy("requestReceiverPose", requestReceiverPose)
    #req = requestReceiverPoseRequest()
    #req.request.data = true
    req = std_msgs.msg.Bool()
    req.data = True
    resp = get_receiver_pose(req)

    if(resp.success.data == True):
        print("Received response")
        print(resp)
