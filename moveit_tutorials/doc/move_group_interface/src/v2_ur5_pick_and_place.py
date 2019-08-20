#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import *
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint


## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group_name2 = "gripper"
    gripper_group = moveit_commander.MoveGroupCommander(group_name2)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = 'object'
    self.robot = robot
    self.scene = scene
    self.group = group
    self.gripper_group = gripper_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  
  def init_upright_path_constraints(self, pose):

    self.upright_constraints = Constraints()
    self.upright_constraints.name = "upright"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = self.eef_link
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.03
    orientation_constraint.absolute_y_axis_tolerance = 0.03
    # orientation_constraint.absolute_z_axis_tolerance = 0.4
    orientation_constraint.absolute_z_axis_tolerance = 0.03 #ignore this axis
    orientation_constraint.weight = 1

    self.upright_constraints.orientation_constraints.append(orientation_constraint)

  def enable_upright_path_constraints(self):
    self.gripper_group.set_path_constraints(self.upright_constraints)

  def disable_upright_path_constraints(self):
    self.gripper_group.set_path_constraints(None)
    
  def go_to_joint_state(self):
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.group.get_current_joint_values()
    joint_goal[1] = -1.5707
    joint_goal[5] = 1.5707

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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL
  
  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to add constraint ..."
    raw_input()
    q = quaternion_from_euler(-1.5707, -1.5707, -1.5707)
    pose_goal_const = geometry_msgs.msg.PoseStamped()
    pose_goal_const.header.frame_id = "robotiq_coupler"
    pose_goal_const.pose.orientation.x =  -0.5 # q[0]
    pose_goal_const.pose.orientation.y =  -0.5 # q[1]
    pose_goal_const.pose.orientation.z =  -0.5 # q[2]
    pose_goal_const.pose.orientation.w =   0.5 # q[3]
    tutorial.init_upright_path_constraints(pose_goal_const)
    tutorial.enable_upright_path_constraints()
    
    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan = tutorial.plan_pose_goal(-1.5707, -1.5707, -1.5707, 0.3224, 0, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.execute_plan(pose_goal_plan)

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan2 = tutorial.plan_pose_goal(-1.5707, -1.5707, -1.5707, 0.36, 0, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.execute_plan(pose_goal_plan2)

    print "============ Press `Enter` to close the gripper ..."
    raw_input()
    tutorial.close_gripper()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = tutorial.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.3722, 0.6)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.execute_plan(pose_goal_plan3)

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = tutorial.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.3722, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.execute_plan(pose_goal_plan3)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    tutorial.detach_box()

    print "============ Press `Enter` to open the gripper ..."
    raw_input()
    tutorial.open_gripper()

    print "============ Press `Enter` to plan a movement using a pose goal ..."
    raw_input()
    pose_goal_plan3 = tutorial.plan_pose_goal(-1.5707, -1.5707, 0, 0.0432, 0.35, 0.5003)

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.execute_plan(pose_goal_plan3)

    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
