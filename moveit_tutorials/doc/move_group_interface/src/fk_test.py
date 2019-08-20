#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import moveit_commander

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

if __name__ == '__main__':
    rospy.init_node('my_node')
    wait_for_time()

    group = moveit_commander.MoveGroupCommander('manipulator')
    rospy.wait_for_service('compute_fk')
    try:
      moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    except rospy.ServiceException, e:
      rospy.logerror("Service call failed: %s"%e)
    fkln = ['robotiq_coupler']
    joint_names = []
    joint_positions = []
    # for i in range(7):
      # joint_names.append('right_arm_j'+str(i)) # your names may vary
      # joint_positions.append(0.8) # try some arbitrary joint angle
    joint_positions.append(0.0)
    joint_positions.append(-1.5707)
    joint_positions.append(0)
    joint_positions.append(-1.5707)
    joint_positions.append(0)
    joint_positions.append(0)
    joint_names.append('elbow_joint') # your names may vary
    joint_names.append('shoulder_lift_joint')
    joint_names.append('shoulder_pan_joint')
    joint_names.append('wrist_1_joint')
    joint_names.append('wrist_2_joint')
    joint_names.append('wrist_3_joint')
    header = Header(0,rospy.Time.now(),"base_link")
    rs = RobotState()
    rs.joint_state.name = joint_names
    rs.joint_state.position = joint_positions
    rospy.loginfo(["FK LOOKUP:", moveit_fk(header, fkln, rs)]) # Lookup the pose
    rospy.loginfo(["FK LOOKUP:", moveit_fk(header, fkln, rs).pose_stamped[0].pose.position.x])

    #### To test, execute joint_positions here ####
    # rospy.loginfo(["POST MOVE:", group.get_current_pose()]) # Verify that the new pose matches your computed pose
