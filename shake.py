#!/usr/bin/env python3
import rospy
from myPlanner import MyRobot
from myPlanner import rad2deg, deg2rad

def shake(robot, shake_delta):
    joint_degrees = rad2deg(robot.get_joints())
    print(joint_degrees)
    for i in range(len(joint_degrees)):
        for j in [-1, 1]:
            joint_degrees[i] += j * shake_delta
            joint_positions = deg2rad(joint_degrees)
            print('desired rads', joint_positions)
            print('current rads', robot.get_joints())
            robot.move_joints(joint_positions , duration=0.1, wait=False)
            rospy.sleep(0.5)  # Sleep for 0.5 seconds between movements

if __name__ == '__main__':
    rospy.init_node('shake', anonymous=True)
    robot = MyRobot()  # Initialize the robot object
    shake(robot, shake_delta=2)
