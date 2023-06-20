#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint


def move_single_arm(arm_1_client,goal_secs = 0.2, arm1_goal = None):
    print("in move arm")

    if arm1_goal is not None:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot1/joint1", "robot1/joint2", "robot1/joint3",
                                        "robot1/joint4", "robot1/joint5", "robot1/joint6"]
        joint_angle = JointTrajectoryPoint()
        joint_angle.positions = arm1_goal
        joint_angle.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(joint_angle)
        print("arm 1 goal: ", arm1_goal)
        arm_1_client.send_goal(goal)
        print("arm 1 command sent")
        rospy.sleep(rospy.Duration(goal_secs))





def main():

    loop_rate = 125

    r = rospy.Rate(loop_rate)
    print("in main")

    arm_1_client = actionlib.SimpleActionClient("/arm1_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
    if not arm_1_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 1 Action Server Not Found")
            rospy.signal_shutdown("Arm 1 Action Server not found")
            sys.exit(1)
    print("Arm 1 Action Server Found")

    current_time = 0.0 
    add_time = 1./ loop_rate
    current_arm = np.radians([-89,-17,36,0,20,1])
    while not rospy.is_shutdown():
        t = current_time * 10
        current_arm[5] = np.radians(10*np.sin(t))
        move_single_arm(arm_1_client=arm_1_client, goal_secs = 0.008,arm1_goal = current_arm)
        # rospy.sleep()
        current_time = current_time + add_time

    # print("in main")

    # rospy.on_shutdown(demo_reset)

    # # Reset demo
    # demo_reset()

    # # Demo start
    # # Forward
    # input("Press enter to continue forward")
    # forward1 = np.radians([90, 30, -30, -60, 0, 90])
    # forward2 = np.radians([90, 30, -30, 60, 0, -90])
    # forward3 = np.radians([-5, 30, -30, 0, 0, 0])
    # forward4 = np.radians([-175, 30, -30, 0, 0, 0]) # Joint 1 is maxed out!
    # backpack.move_arm(forward1, forward2, forward3, forward4, 5)

    # # Look left
    # input("Press enter to continue look_left")
    # left1 = np.radians([90, 30, -30, -60, -45, 90])
    # left2 = np.radians([90, 30, -30, 60, 45, -90])
    # left3 = np.radians([-5, 30, -30, 0, -45, 0])
    # left4 = np.radians([-175, 30, -30, 0, 45, 0]) # Joint 1 is maxed out!
    # backpack.move_arm(left1, left2, left3, left4, 0.5)

    # # Look right
    # input("Press enter to continue look_right")
    # right1 = np.radians([90, 30, -30, -60, 45, 90])
    # right2 = np.radians([90, 30, -30, 60, -45, -90])
    # right3 = np.radians([-5, 30, -30, 0, 45, 0])
    # right4 = np.radians([-175, 30, -30, 0, -45, 0]) # Joint 1 is maxed out!
    # backpack.move_arm(right1, right2, right3, right4, 1)

    # # Back to forward
    # input("Press enter to continue look_forward")
    # backpack.move_arm(forward1, forward2, forward3, forward4, 0.5)

    # # Converge
    # input("Press enter to continue converge")
    # converge1 = np.radians([120, 45, 0, 45, -45, 0])
    # converge2 = np.radians([60, 45, 0, -45, -45, 0])
    # converge3 = np.radians([-30, 45, 0, -45, -45, 0])
    # converge4 = np.radians([-150, 45, 0 ,45, -45, 0])
    # backpack.move_arm(converge1, converge2, converge3, converge4, 5)

    # # Back to rest
    # input("Press enter to continue rest")
    # rospy.signal_shutdown(None)

if __name__ == '__main__':
    rospy.init_node("sinewave_wrist")

    # backpack = Backpack()

    main()