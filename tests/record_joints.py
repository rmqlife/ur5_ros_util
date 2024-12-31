import rospy
from myPlanner import MyRobot, MyConfig
import sys

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

if __name__=="__main__":
    rospy.init_node('record poses', anonymous=True)

    robot = MyRobot()
    joints = robot.get_joints()
    print(f"Joints: {joints}")
        
    # Create a JointConfig instance
    joint_configs = MyConfig("../config/saved_joints.json")
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