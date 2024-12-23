#!/usr/bin/env python3
import json
from .myRobot import MyRobot  # Import your MyRobot class
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

def get_next_pose_name(config_dict):
    """Generate the next pose name like pose1, pose2, ... based on existing configurations."""
    # Find the highest number in existing pose names like pose1, pose2, ..., poseN
    highest_pose = 0
    for name in config_dict.keys():
        if name.startswith("pose"):
            try:
                pose_num = int(name[4:])
                highest_pose = max(highest_pose, pose_num)
            except ValueError:
                continue
    return f"pose{highest_pose + 1}"

if __name__ == "__main__":
    
    rospy.init_node('record poses', anonymous=True)

    robot = MyRobot()
    joints = robot.get_joints()
    print(f"Joints: {joints}")
        
    # Create a JointConfig instance
    joint_configs = MyConfig("config/restricted_joints.json")
    print(f"Existing configurations: {joint_configs.config_dict}")
    
    # Check if a configuration name is provided as a command-line argument
    if len(sys.argv) > 1:
        # Use the provided name
        config_name_to_save = sys.argv[1]
        joint_configs.save(config_name_to_save, joints)
        print(f"Saved joint configuration '{config_name_to_save}': {joints}")
    else:
        # No name provided, generate the next pose name
        new_config_name = get_next_pose_name(joint_configs.config_dict)
        joint_configs.save(new_config_name, joints)
        print(f"No configuration name provided. Generated '{new_config_name}' and saved joint configuration: {joints}")
