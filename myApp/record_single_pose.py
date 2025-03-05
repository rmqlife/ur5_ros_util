from myPlanner import MyBag, init_robot_with_ik, SE3_to_pose
import rospy
import sys

def main():
    # Check if pose name is provided
    if len(sys.argv) != 2:
        print("Usage: python record_single_pose.py <pose_name>")
        sys.exit(1)

    pose_name = sys.argv[1]
    
    try:
        rospy.init_node('record_single_pose', anonymous=True)
        
        # Initialize the data saver
        mybag = MyBag("../llm/key_poses.json")
        
        # Initialize robot
        robot = init_robot_with_ik()
        
        # Record the pose with the specified name
        current_pose = SE3_to_pose(robot.get_pose())
        
        mybag.record(pose_name, current_pose)
        
        print(f"Successfully recorded pose '{pose_name}'")
        
    except rospy.ROSInterruptException:
        print("Program interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
        sys.exit(1)
    finally:
        rospy.signal_shutdown("Recording complete")

if __name__ == '__main__':
    main()
        
        