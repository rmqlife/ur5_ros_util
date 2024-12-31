#!/usr/bin/env python3

import rospy
from myPlanner import MyRobot
from myPlanner import MyConfig  # Import the MyConfig class
import sys

def main(config_name):
    """
    Initializes the robot and loads the specified joint configuration to reset the robot's pose.
    """
    try:
        rospy.init_node('reset', anonymous=True)
        my_robot = MyRobot()  # Initialize the robot object

        # Create a JointConfig instance
        joint_configs = MyConfig('../config/saved_joints.json')

        # Check if the reset pose is already saved, and load it if available
        if joint_configs.get(config_name):
            reset_joints = joint_configs.get(config_name)
            print("Loaded reset pose:", reset_joints)
            current_joints = my_robot.get_joints()
            print('current joints', current_joints)
            my_robot.move_joints_smooth(reset_joints, coef=1.5, joint_thresh=10, wait=False)
        else:
            print(f"Configuration '{config_name}' not found in joint configurations.")
    
    except rospy.ROSInterruptException:
        print('ROS Interrupt Exception occurred. Exiting...')
    except Exception as e:
        print(f'An error occurred: {e}')

if __name__ == '__main__':
    main(config_name=sys.argv[1])
