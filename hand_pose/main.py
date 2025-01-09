import numpy as np
import sys
import rospy
from read_hand import hand_pose,smooth_trajectory,find_transformation_vectors,_4points_to_3d, draw_movingframe
from spatialmath import SE3
from myPlanner.pose_util import pose_to_SE3, SE3_to_pose
from matplotlib import pyplot as plt


def tracking_data_to_actions(csv_path, visualize=False):
    pd_data = hand_pose(csv_path)
    data = pd_data.get_hand_pose(2, 50)
    keypoints = pd_data.get_keypoints(data,list=[0,5,9])
    new_keypoints = pd_data.get_keypoints(data,list=[4,8,2,9])
    # gripper_scale = pd_data.get_simulated_gripper_size(data)
    # print(gripper_scale)
    actions_SE3 = []
    for i in range(len(keypoints)-1):
        vector_list1, point1  = _4points_to_3d(new_keypoints[0],find_transformation=True)
        vector_list2, point2  = _4points_to_3d(new_keypoints[i+1],find_transformation=True)
        T_ori = find_transformation_vectors(vector_list1, vector_list2, point1, point2)

        action = SE3(T_ori)
        actions_SE3.append(action)
        action.printline()
    # SE3_poses = smooth_trajectory(SE3_poses)
    # print(se)
    if visualize:
        draw_movingframe(actions_SE3, new_keypoints)
    return actions_SE3

def ee_actions_to_robot_traj(actions, pose):
    pose_se3 = pose_to_SE3(pose)
    poses = []
    for action in actions:
        pose_se3_new =  action # right multiply
        poses.append(SE3_to_pose(pose_se3_new))
    return poses


if __name__ == "__main__":
    actions = tracking_data_to_actions("/home/rmqlife/work/ur5_ros_utils/hand_pose/test.csv", visualize=False)

    dry_run = True
    rospy.init_node('replay_hand_actions')
    if dry_run:

        # robot.myIK.show_traj(actions, loop=True)
        from myPlanner import init_robot_with_ik, visualize_poses
        robot = init_robot_with_ik()
        init_pose = robot.get_pose()
        poses = ee_actions_to_robot_traj(actions, init_pose)
        ax = visualize_poses(init_pose, label='init pose', autoscale=False, ax=None)
        visualize_poses(poses, label="points to plan", color='y', autoscale=False, ax=ax)
        plt.show()
    else:
        import rospy
        rospy.init_node('replay_hand_actions')
        # object move
        from myPlanner import init_robot_with_ik
        robot = init_robot_with_ik()
        for action in actions: 
            print(action)
            # robot.step_in_ee(action, wait=False)
            # rospy.sleep(0.05)
