#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String, Header, ColorRGBA
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from moveit_msgs.srv import *

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_ur5_robot',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_name2 = "gripper"
    gripper_group = moveit_commander.MoveGroupCommander(group_name2)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    rospy.sleep(0.5)

    rospy.wait_for_service('compute_fk')
    try:
      self.moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    except rospy.ServiceException, e:
      rospy.logerror("Service call failed: %s"%e)

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    rospy.sleep(0.5)

    self.marker_publisher = marker_publisher
    self.fkln = ['robotiq_coupler']
    self.joint_names = []
    self.joint_positions = []
    self.rs = RobotState()
    self.marker2 = Marker()

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.box_name = 'object'
    self.robot = robot
    self.scene = scene
    self.group = group
    self.gripper_group = gripper_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.group.get_current_joint_values()
    joint_goal[1] = -1.5707
    joint_goal[3] = -1.5707

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def close_gripper(self):
    joint_goal = self.gripper_group.get_current_joint_values()
    joint_goal[0] = 0.6
    self.gripper_group.go(joint_goal, wait=True)
    self.gripper_group.stop()
    current_joints = self.gripper_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def open_gripper(self):
    joint_goal = self.gripper_group.get_current_joint_values()
    joint_goal[0] = 0.0
    self.gripper_group.go(joint_goal, wait=True)
    self.gripper_group.stop()
    current_joints = self.gripper_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def plan_pose_goal(self, R, P, Y, x, y, z):

    q = quaternion_from_euler(R, P, Y)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    # print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.group.set_pose_target(pose_goal)

    plan = self.group.plan()

    return plan

  def go_to_pose_goal(self, R, P, Y, x, y, z):
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    q = quaternion_from_euler(R, P, Y)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory)

  def execute_plan(self, plan):
    group = self.group
    group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():

      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False

  def add_box(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.2
    self.scene.add_box("box", box_pose, size=(0.2, 0.4, 0.4))

    box_pose2 = geometry_msgs.msg.PoseStamped()
    box_pose2.header.frame_id = "base_link"
    box_pose2.pose.orientation.w = 1.0
    box_pose2.pose.position.x = 0
    box_pose2.pose.position.y = 0.5
    box_pose2.pose.position.z = 0.2
    self.scene.add_box("box2", box_pose2, size=(0.4, 0.2, 0.4))

    box_pose3 = geometry_msgs.msg.PoseStamped()
    box_pose3.header.frame_id = "base_link"
    box_pose3.pose.orientation.w = 1.0
    box_pose3.pose.position.x = 0.2
    box_pose3.pose.position.y = 0.2
    box_pose3.pose.position.z = 0.4
    self.scene.add_box("box3", box_pose3, size=(0.1, 0.1, 0.8))

    # Planes nao aparecem
    plane1_pose = geometry_msgs.msg.PoseStamped()
    plane1_pose.header.frame_id = "base_link"
    plane1_pose.pose.orientation.w = 1.0
    plane1_pose.pose.position.x = 0
    plane1_pose.pose.position.y = 0
    plane1_pose.pose.position.z = 0
    self.scene.add_plane("plane1", plane1_pose)

    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "base_link"
    object_pose.pose.orientation.w = 1.0
    object_pose.pose.position.x = 0.5
    object_pose.pose.position.y = 0
    object_pose.pose.position.z = 0.5
    self.scene.add_box("object", object_pose, size=(0.02, 0.02, 0.2))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link
    scene.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def show_path_in_rviz(self):
      self.marker2.header.frame_id = "base_link"
      self.marker2.type = self.marker2.LINE_STRIP
      self.marker2.action = self.marker2.MODIFY
      self.marker2.scale = Vector3(0.008, 0.009, 0.1)
      self.marker2.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

      rospy.loginfo(self.marker2)
      self.marker_publisher.publish(self.marker2)

  def visualize_plan(self, plan):

      limit = len(plan.joint_trajectory.points)
      rospy.loginfo("Quantidade: " + str(limit))

      for inn in range(limit):
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[0])
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[1])
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[2])
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[3])
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[4])
         self.joint_positions.append(plan.joint_trajectory.points[inn].positions[5])

         self.joint_names.append('shoulder_pan_joint') # your names may vary
         self.joint_names.append('shoulder_lift_joint')
         self.joint_names.append('elbow_joint')
         self.joint_names.append('wrist_1_joint')
         self.joint_names.append('wrist_2_joint')
         self.joint_names.append('wrist_3_joint')

         header = Header(inn,rospy.Time.now(),"base_link")

         self.rs.joint_state.name = self.joint_names
         self.rs.joint_state.position = self.joint_positions

         self.marker2.points.append(self.moveit_fk(header, self.fkln, self.rs).pose_stamped[0].pose.position)

         # rospy.loginfo(["FK LOOKUP:", self.moveit_fk(header, self.fkln, self.rs)]) # Lookup the pose
         self.show_path_in_rviz()

def main():
  try:
    ur5_robot = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    ur5_robot.go_to_joint_state()

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    ur5_robot.add_box()

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan = ur5_robot.plan_pose_goal(-1.5707, -1.5707, -1.5707, 0.3224, 0, 0.5003)
    ur5_robot.visualize_plan(pose_goal_plan)
    rospy.loginfo(pose_goal_plan)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur5_robot.execute_plan(pose_goal_plan)

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan2 = ur5_robot.plan_pose_goal(-1.5707, -1.5707, -1.5707, 0.36, 0, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur5_robot.execute_plan(pose_goal_plan2)

    print "============ Press `Enter` to close the gripper ..."
    raw_input()
    ur5_robot.close_gripper()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    ur5_robot.attach_box()

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = ur5_robot.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.3722, 0.6)
    ur5_robot.visualize_plan(pose_goal_plan3)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur5_robot.execute_plan(pose_goal_plan3)

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = ur5_robot.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.3722, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur5_robot.execute_plan(pose_goal_plan3)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    ur5_robot.detach_box()

    print "============ Press `Enter` to open the gripper ..."
    raw_input()
    ur5_robot.open_gripper()

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = ur5_robot.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.35, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur5_robot.execute_plan(pose_goal_plan3)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
