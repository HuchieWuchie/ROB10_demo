import rospy
import actionlib

from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from iiwa_msgs.msg import JointPosition, Spline, SplineSegment, MoveAlongSplineAction, MoveToJointPositionAction, MoveToJointPositionGoal, MoveAlongSplineGoal
from iiwa_msgs.srv import SetPTPJointSpeedLimits, SetEndpointFrame, SetPTPCartesianSpeedLimits

def setEndpointFrame(frame_id = "iiwa_link_ee"):
	print("Setting endpoint frame to \"", frame_id, "\"...")
	set_endpoint_frame_client = rospy.ServiceProxy("/iiwa/configuration/setEndpointFrame", SetEndpointFrame)
	response = set_endpoint_frame_client.call(frame_id)

	if not response.success:
		print("Service call returned error: ", response.error)
		return False

	return True

def setPTPJointSpeedLimits(joint_rel_vel = 1.0, joint_rel_acc = 1.0):
    print("Setting PTP joint speed limits...")
    set_ptp_joint_speed_client = rospy.ServiceProxy("/iiwa/configuration/setPTPJointLimits", SetPTPJointSpeedLimits)

    response = set_ptp_joint_speed_client.call(joint_rel_vel, joint_rel_acc)


    if not response.success:
        print("Service call returned error: ", response.error)
        return False


    return True

def setPTPCartesianSpeedLimits(max_cart_vel = 1.0, max_cart_acc = 1.0,
                                max_cart_jerk = 1.0, max_orien_vel = 1.0,
                                max_orien_acc = 1.0, max_orien_jerk = 1.0):
    print("Setting PTP Cartesian speed limits...")
    set_ptp_cartesian_speed_client = rospy.ServiceProxy("/iiwa/configuration/setPTPCartesianLimits", SetPTPCartesianSpeedLimits)

    response = set_ptp_cartesian_speed_client(max_cart_vel, max_cart_acc, max_cart_jerk,
                                                max_orien_vel, max_orien_acc, max_orien_jerk)

    if not response.success:
        print("Service call returned error: ", response.error);
        return False

    return True

def getSplineSegment (x, y, z, qx, qy, qz, qw, type = 0):
    """ Creates a spline segment used by KUKA IIWA planning """

    segment = SplineSegment()

    segment.type = type;

    segment.point.poseStamped.header.frame_id = "iiwa_link_0"

    segment.point.poseStamped.pose.position.x = x
    segment.point.poseStamped.pose.position.y = y
    segment.point.poseStamped.pose.position.z = z

    segment.point.poseStamped.pose.orientation.x = qx
    segment.point.poseStamped.pose.orientation.y = qy
    segment.point.poseStamped.pose.orientation.z = qz
    segment.point.poseStamped.pose.orientation.w = qw

    segment.point.redundancy.status = -1
    segment.point.redundancy.turn = -1

    return segment

def execute_spline_trajectory(plan):
    print("Executing trajectory with spline motion")

    # Compute cartesian poses for each point in joint trajectory

    rospy.wait_for_service('/iiwa/compute_fk')
    moveit_fk = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

    cartesian_poses = []

    end_effector_link = ['iiwa_link_ee']
    joint_names = []
    joint_positions = []
    for joint_position in plan.joint_trajectory.points:
        for i in range(7):
          joint_names.append('iiwa_joint_'+str(i + 1)) # joint names, see /iiwa/joint_state
          print(joint_position)
          joint_positions.append(joint_position.positions[i])
        header = Header(0,rospy.Time.now(),"iiwa_link_0") # base of IIWA
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = joint_positions
        fk_result = moveit_fk(header, end_effector_link, rs) # Lookup the pose

        x, y, z = fk_result.pose_stamped[0].pose.position.x, fk_result.pose_stamped[0].pose.position.y, fk_result.pose_stamped[0].pose.position.z
        qx, qy, qz, qw = fk_result.pose_stamped[0].pose.orientation.x, fk_result.pose_stamped[0].pose.orientation.y, fk_result.pose_stamped[0].pose.orientation.z, fk_result.pose_stamped[0].pose.orientation.w

        cartesian_poses.append([x, y, z, qx, qy, qz, qw])

    # Assemble spline with SPL motions

    spline_motion = MoveAlongSplineGoal()

    for pose in cartesian_poses:
        x, y, z, qx, qy, qz, qw = pose
        spline_motion.spline.segments.append(getSplineSegment(x, y, z, qx, qy, qz, qw, type = 0))

    # Send and execute
    spline_motion_client = actionlib.SimpleActionClient("/iiwa/action/move_along_spline", MoveAlongSplineAction)

    print("Waiting for action servers to start...")
    spline_motion_client.wait_for_server()

    spline_motion_client.send_goal(spline_motion)
    spline_motion_client.wait_for_result()
