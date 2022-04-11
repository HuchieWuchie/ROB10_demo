#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from moveit_scripts.srv import *
import numpy as np
from tf.transformations import *
#TODO check in what order are quaternion numpy arrays orders - xyzw or wxyz

def get_random_transformation():
    transform = geometry_msgs.msg.Transform()
    transform.translation.x = np.random.uniform(low=0.5, high=1.5)
    transform.translation.y = np.random.uniform(low=0.5, high=1.5)
    transform.translation.z = np.random.uniform(low=0.5, high=1.5)

    roll = np.random.uniform(low=0.0, high=3.1416)
    pitch = np.random.uniform(low=0.0, high=3.1416)
    yaw = np.random.uniform(low=0.0, high=3.1416)
    q = quaternion_from_euler(roll, pitch, yaw)
    # normalize quaternion
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    # turn numpy array q to geometry_msgs
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    return transform

def temp_handle_get_goal_ee(req):
    print("handle")
    print(req)
    print("=================================")

    res = GetGoalEeTransformationResponse()
    res.goal_ee = get_random_transformation()
    res.success.data = True
    print(res)
    return res

def msg_to_matrix(pose):
    #http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
    q = pose.rotation
    #https://answers.ros.org/question/324354/how-to-get-rotation-matrix-from-quaternion-in-python/
    transformation = quaternion_matrix([q.w, q.x, q.y, q.z])
    #print(transformation)

    #rot = R.from_quat(pose.orientation)
    #transformation[0, :] = rot[0, :]
    #transformation[1, :] = rot[1, :]
    #transformation[2, :] = rot[2, :]
    #print(transformation)

    transformation[0,3] = pose.translation.x
    transformation[1,3] = pose.translation.y
    transformation[2,3] = pose.translation.z
    #print(transformation)

    #q = pose.orientation
    #rpy = tf.transformations.euler_from_quaternion(q)
    return transformation

def handle_get_goal_ee(req):
    grasp_mat =msg_to_matrix(req.grasp)
    centroid_mat =msg_to_matrix(req.centroid)
    goal_centroid_mat =msg_to_matrix(req.goal_centroid)
    grasp_mat_inverse = inverse_matrix(grasp_mat)
    ee_centroid_mat = np.matmul(grasp_mat_inverse, centroid_mat)
    print(ee_centroid_mat)

    ee_centroid_mat_inverse = inverse_matrix(ee_centroid_mat)
    goal_ee_mat = np.matmul(centroid_mat, ee_centroid_mat_inverse)
    print(goal_ee_mat)
    goal_q = quaternion_from_matrix(goal_ee_mat)

    res = GetGoalEeTransformationResponse()
    res.success.data = True

    res.goal_ee.translation.x = goal_ee_mat[0,3]
    res.goal_ee.translation.y = goal_ee_mat[1,3]
    res.goal_ee.translation.z = goal_ee_mat[2,3]
    res.goal_ee.rotation.x = goal_q[0]
    res.goal_ee.rotation.y = goal_q[1]
    res.goal_ee.rotation.z = goal_q[2]
    res.goal_ee.rotation.w = goal_q[3]
    print(res)
    return res

if __name__ == '__main__':
    rospy.init_node('get_goal_ee_transformation_server_node', anonymous=True)
    server = rospy.Service('get_goal_ee_transformation', GetGoalEeTransformation, handle_get_goal_ee)

    print("server is ready")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("except")
        pass

"""
geometry_msgs/Transform grasp
geometry_msgs/Transform centroid
geometry_msgs/Transform goal_centroid
---
geometry_msgs/Transform goal_ee
std_msgs/Bool success
"""
