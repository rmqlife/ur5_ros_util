import pickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
from myEnv import MyStates, ArucoState
import json


def se3_traj_to_2d(se3_poses):
    """Convert a list of SE(3) poses to 2D positions."""
    return np.array([(pose.t[0], pose.t[1]) for pose in se3_poses])



def visualize_trajectory(states):
    if not states:
        print("No states to visualize.")
        return

    # Extract trajectories
    robot_trajectory = [state.robot_pose for state in states]
    marker_trajectories = {}

    # Convert robot trajectory to 2D
    robot_trajectory_2d = se3_traj_to_2d(robot_trajectory)

    marker_info = json.load(open('marker_info.json', 'r'))
    marker_names = {marker_info[key]["marker_id"]: key for key in marker_info}
    print(marker_names)

    for state in states:
        for marker_id, pose in state.marker_poses.items():
            if marker_id not in marker_trajectories:
                marker_trajectories[marker_id] = []
            marker_trajectories[marker_id].append(pose)

    # Plot robot trajectory
    robot_x = robot_trajectory_2d[:, 0]
    robot_y = robot_trajectory_2d[:, 1]
    plt.figure(figsize=(10, 6))

    plt.plot(robot_x, robot_y, label='Robot Trajectory', marker='o')

    # # Plot marker trajectories
    for marker_id, trajectory in marker_trajectories.items():
        marker_trajectory_2d = se3_traj_to_2d(trajectory)
        marker_x = marker_trajectory_2d[:, 0]
        marker_y = marker_trajectory_2d[:, 1]
        marker_name = marker_names[marker_id]
        plt.plot(marker_x, marker_y, label=f'Marker {marker_id} {marker_name}', marker='x')

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Trajectory Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()

def states_prompt(states):
    if not states:
        print("No states available.")
        return

    # Extract robot trajectory
    robot_trajectory = [state.robot_pose for state in states]
    robot_trajectory_2d = se3_traj_to_2d(robot_trajectory)

    # Print robot trajectory 2D
    print("Robot Trajectory 2D:")
    for x, y in robot_trajectory_2d:
        print(f"({x:.3f}, {y:.3f})")

    # Extract and print marker trajectories
    marker_trajectories = {}
    for state in states:
        for marker_id, pose in state.marker_poses.items():
            if marker_id not in marker_trajectories:
                marker_trajectories[marker_id] = []
            marker_trajectories[marker_id].append(pose)

    marker_info = json.load(open('marker_info.json', 'r'))
    marker_names = {marker_info[key]["marker_id"]: key for key in marker_info}

    print("Marker Trajectories 2D:")
    for marker_id, trajectory in marker_trajectories.items():
        marker_trajectory_2d = se3_traj_to_2d(trajectory)
        marker_name = marker_names.get(marker_id, "Unknown")
        print(f"Marker {marker_id} ({marker_name}):")
        for x, y in marker_trajectory_2d:
            print(f"({x:.3f}, {y:.3f})")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        mystates = MyStates(filename)
        states = mystates.states
        print(f"Loaded {len(states)} states")
        visualize_trajectory(mystates.states)
        # states_prompt(mystates.states)
    else:
        print("Usage: python display_states.py <filename>")