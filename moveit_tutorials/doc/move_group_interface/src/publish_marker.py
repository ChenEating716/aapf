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

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    rospy.sleep(0.5)

    marker2 = Marker()
    marker2.header.frame_id = "base_link"
    marker2.type = marker2.LINE_STRIP
    marker2.action = marker2.MODIFY
    # marker2.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    marker2.scale = Vector3(0.008, 0.009, 0.1)
    marker2.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

    marker2.points = []
    first_point = Point()
    first_point = Point(0, 0, 0)
    marker2.points.append(first_point)

    second_point = Point()
    second_point = Point(1, 0, 2)
    marker2.points.append(second_point)

    third_point = Point()
    third_point = Point(0, 1, 2)
    marker2.points.append(third_point)

    marker_publisher.publish(marker2)
