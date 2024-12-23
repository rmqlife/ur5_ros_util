import numpy as np
from myPlanner.pose_util import *
from myPlanner import circle_points, init_robot_with_ik
import rospy

if __name__ == "__main__":
    dry_run = True
    rospy.init_node('test_move', anonymous=False)
    
    robot = init_robot_with_ik()

    init_pose = robot.get_pose()

    print("init pose at", init_pose)

    goal_pose = init_pose.copy()
    goal_pose[2] -= 0.1
    waypoints = [init_pose, goal_pose]
    
    waypoints = circle_points(init_pose[:3], radius=0.1, num_points=50)

    poses = []
    for _ in range(5):
        poses += waypoints
    robot.goto_poses(poses, dry_run=False, coef=3)
