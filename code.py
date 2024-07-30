#!/usr/bin/env python

import rospy
import time
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        response = set_mode_service(custom_mode=mode)
        return response.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr("Failed to set mode: %s" % e)
        return False

def arm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        response = arm_service(True)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Failed to arm: %s" % e)
        return False

def navigate_waypoints(waypoints):
    pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20)

    for point in waypoints:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]

        for _ in range(100):
            pose.header.stamp = rospy.Time.now()
            pose_pub.publish(pose)
            rate.sleep()

        while not rospy.is_shutdown():
            distance = ((current_state.pose.pose.position.x - point[0]) ** 2 +
                        (current_state.pose.pose.position.y - point[1]) ** 2 +
                        (current_state.pose.pose.position.z - point[2]) ** 2) ** 0.5

            if distance < 0.2:
                break

            pose.header.stamp = rospy.Time.now()
            pose_pub.publish(pose)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drone_navigation', anonymous=True)
    rospy.Subscriber('mavros/state', State, state_cb)

    waypoints = [
        [0, 0, 2],
        [2, 0, 2],
        [2, 2, 2],
        [0, 2, 2],
        [0, 0, 2]
    ]

    while not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rospy.sleep(1)

    rospy.loginfo("FCU connected")

    set_mode('OFFBOARD')
    arm()

    navigate_waypoints(waypoints)
