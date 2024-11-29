#!/usr/bin/env python3
import json
from myRobot import MyRobot  # Import your MyRobot class
import sys  # Import the sys module for command-line arguments
import rospy

class MyConfig:
    def __init__(self, file_path):
        self.file_path = file_path
        self.load()  # Initialize the config_dict with loaded configurations
    
    def save(self, config_name, joint_positions):
        # Create a dictionary to store the joint positions with a name
        # Open the JSON file for writing
        self.config_dict[config_name]=list(joint_positions)
        print(self.config_dict)
        with open(self.file_path, 'w') as file:
            # Append the new joint configuration to the file
            json.dump(self.config_dict, file, indent=4)

    def load(self):
        try:
            # Open the JSON file for reading
            with open(self.file_path, 'r') as file:
                self.config_dict = json.load(file)
        except Exception as e:
            # Handle the case where the file does not exist
            print(f"Configuration file '{self.file_path}' not found.")
            self.config_dict ={}

    def get(self, name):
        return self.config_dict[name]

if __name__ == '__main__':
    try:
        # Initialize the ROS node and the robot
        rospy.init_node('ur5_shake_test', anonymous=True)
        
        # Create a JointConfig instance
        joint_configs = MyConfig("config/robot_joints.json")
        print(joint_configs.config_dict)
        

        robot = MyRobot() 
        joints = robot.get_joints()

        print(joints)
        # # Check if a configuration name is provided as a command-line argument
        if len(sys.argv) > 1:
            # Specify the configuration name to save
            config_name_to_save = sys.argv[1]  # Use the provided command-line argument
            joint_configs.save(config_name_to_save, joints)
            print(f"Saved joint configuration '{config_name_to_save}': {joints}")
        else:
            print("Usage: ./myConfig.py <config_name>")

    except rospy.ROSInterruptException:
        pass