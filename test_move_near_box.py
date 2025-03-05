import rospy
from visual_tracking.myEnv import MyEnv
from myPlanner import se3_to_str
import numpy as np
from llm.simple_tcp import find_marker, move_near_box_direction

def move_towards_circle():
    rospy.init_node('test_move_near_box_direction')
    env = MyEnv(is_publisher=False)
    rospy.sleep(0.5)

    # Define marker ID and z_offset
    marker_id = 3  # Example marker ID
    z_offset = 0.1  # Height offset above the box

    # Find the goal pose using the marker ID

    # Define the circle center
    current_pose = env.robot.get_pose().t  # Assuming goal_pose contains [x, y, z, ...]
    circle_center = np.array([tcp_x, tcp_y, tcp_z - z_offset])

    # Calculate the direction towards the circle center
    direction = circle_center - current_pose
    direction = direction / np.linalg.norm(direction)  # Normalize the direction

    distance = 0.2  # Distance to move near the circle

    # Move near the circle in the specified direction
    final_pose = move_near_box_direction(env, marker_id, direction, distance, z_offset)
    print(f"Final robot pose after moving towards the circle: {se3_to_str(final_pose)}")

if __name__ == "__main__":
    move_towards_circle() 