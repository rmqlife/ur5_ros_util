# find the tcp pose in the robot base frame
import rospy
from visual_tracking.myEnv import MyEnv
import cv2
from myPlanner import Rt_to_SE3, same_position, se3_to_str
import numpy as np
import copy


def move_z(env, z):
    robot_pose = env.robot.get_pose().copy()
    robot_pose.t[2]+=z
    env.robot.goto_pose(robot_pose, wait=True)

def find_marker(env, marker_id):
    state, _ = env.get_state()
    while state is None or not marker_id in state.marker_poses:
        move_z(env=env, z=0.1)
        state,_ = env.get_state()
    # deep copy this    
    return copy.deepcopy(state.marker_poses[marker_id])

def move_close_to_marker(env, marker_id, offset=[0,0,0.18]):
    while True:
        state, _ = env.get_state()
        marker_pose = state.marker_poses[marker_id]
        robot_pose = state.robot_pose.copy()
        robot_pose.t = marker_pose.copy().t
        robot_pose.t += np.array(offset)
        if same_position(robot_pose, env.robot.get_pose()):
            print("already move above the marker")
            break
        env.robot.goto_pose(robot_pose, wait=False)
        rospy.sleep(0.1)

def push_box_to_goal(env, marker_id, goal_pose, push_distance=0.03, z_offset=0.13, max_attempts=20):
    """
    Push a box with marker_id to a goal_pose
    
    Args:
        env: The environment object
        marker_id: ID of the marker on the box
        goal_pose: Target pose for the box
        push_distance: How far to push in each step
        z_offset: Height offset from marker for robot positioning
        max_attempts: Maximum number of push attempts before giving up
    """
    print(f"Starting to push box with marker {marker_id} to goal")
    box_size = 0.10
    attempts = 0
    last_distance = float('inf')
    
    # Keep pushing until the box reaches the goal
    while attempts < max_attempts:
        attempts += 1
        
        # Get current marker pose
        try:
            current_pose = find_marker(env, marker_id)
        except Exception as e:
            print(f"Error finding marker: {e}. Moving up to try again.")
            move_z(env, 0.1)
            continue
            
        # Calculate current distance to goal
        current_distance = np.linalg.norm(current_pose.t - goal_pose.t)
        print(f"Attempt {attempts}: Distance to goal: {current_distance:.3f}m")

        # Check if we've reached the goal
        if same_position(current_pose, goal_pose, t_threshold=0.02, rad_threshold=3.14):
            print(f"Success! Box reached goal position.")
            print(f"Current: {se3_to_str(current_pose)}")
            print(f"Goal: {se3_to_str(goal_pose)}")
            return True
            
        # Check if we're getting closer
        if abs(current_distance - last_distance) < 0.005 and attempts > 3:
            print(f"Box seems stuck (distance change: {abs(current_distance - last_distance):.4f}m)")
            # Try a slightly different approach angle
            offset_angle = (attempts % 3 - 1) * 0.2  # -0.2, 0, or 0.2 radians
        else:
            offset_angle = 0
            
        last_distance = current_distance
            
        # Calculate direction vector from current position to goal
        direction = goal_pose.t - current_pose.t
        direction[2] = 0  # Keep movement in the XY plane
        
        # Apply small angle offset if needed
        if offset_angle != 0:
            rotation_matrix = np.array([
                [np.cos(offset_angle), -np.sin(offset_angle), 0],
                [np.sin(offset_angle), np.cos(offset_angle), 0],
                [0, 0, 1]
            ])
            direction = rotation_matrix.dot(direction)
            
        direction_magnitude = np.linalg.norm(direction)
        if direction_magnitude < 1e-6:
            print("Direction vector too small, moving up and retrying")
            move_z(env, 0.1)
            continue
            
        direction = direction / direction_magnitude
        
        # Adjust push distance based on distance to goal
        adaptive_push = min(push_distance, current_distance * 0.7)
        
        # Calculate position to approach the box from
        approach_vector = -direction * box_size  # Approach from the opposite direction of desired push
        approach_offset_adjusted = np.array([approach_vector[0], approach_vector[1], z_offset])
        
        # Move to the approach position
        print(f"Moving to approach position (angle offset: {offset_angle:.2f} rad)")
        move_close_to_marker(env, marker_id, offset=approach_offset_adjusted)
        
        # Now push forward
        print(f"Pushing box toward goal (push distance: {adaptive_push:.3f}m)")
        robot_pose = env.robot.get_pose().copy()
        push_pose = robot_pose.copy()
        push_pose.t += direction * (box_size + adaptive_push)
        env.robot.goto_pose(push_pose, wait=True)
        
        # Move up to avoid hitting the box when repositioning
        move_z(env, 0.15)
        
        # Small delay to let things settle
        rospy.sleep(0.5)
    
    print(f"Failed to reach goal after {max_attempts} attempts")
    return False


if __name__ == "__main__":
    rospy.init_node('simple_tcp')
    env = MyEnv(is_publisher=False)
    rospy.sleep(0.5)
    box_id = 3
    goal_id = 6
    goal_pose = find_marker(env, marker_id=goal_id)
    print(f"found marker {goal_id} at {se3_to_str(goal_pose)}")
    
    # Define a goal pose (for example, 0.3m in the x direction from current position)
    goal_pose = goal_pose.copy()
    # Push the box to the goal
    push_box_to_goal(env, marker_id=box_id, goal_pose=goal_pose)
