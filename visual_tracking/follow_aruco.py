import numpy as np
from spatialmath import SE3
from myPlanner import init_robot_with_ik, MyBag
from myPlanner.pose_util import *
from mySensor import MyImageSaver
from myApp import key_map, lookup_action
import rospy
import cv2
from aruco_util import get_aruco_poses
from mySensor import load_intrinsics
from myHandEye import MyHandEye

MARKER_SIZE = 0.0258
framedelay = 1000//20

def clear_se3_axis(se3_pose, pattern="111000"):
    # se3 to euler_6d
    euler_pose = se3_to_euler6d(se3_pose)
    for i, p in enumerate(pattern):
        if p == '0':
            euler_pose[i] = 0
    return euler_to_se3(euler_pose)

def relative_pose(pose1, pose2, pattern="111000"):
    # only return the translation part
    pose1 = clear_se3_axis(pose1, pattern)
    pose2 = clear_se3_axis(pose2, pattern)       
    return pose2 * pose1.inv()

def remove_small_motion(move):
    no_move = SE3(np.eye(4))
    rotation_norm = np.linalg.norm(move.R - np.eye(3))  # Deviation from identity rotation
    translation_norm = np.linalg.norm(move.t)
    if rotation_norm > 0.1 and translation_norm > 0.010:
        return move
    elif rotation_norm > 0.1:
        return Rt_to_SE3(move.R, no_move.t)
    elif translation_norm > 0.010:
        return Rt_to_SE3(no_move.R, move.t)
    return no_move

def euler_to_se3(euler_6d):
    t = euler_6d[:3]
    r = euler_6d[3:]
    se3_object = SE3.Trans(t) * SE3.RPY(r, unit='rad')
    return se3_object

def se3_to_euler6d(se3_object):
    translation = se3_object.t
    rotation = se3_object.rpy(unit='rad')
    euler_6d = list(translation) + list(rotation)
    return euler_6d


if __name__ == "__main__":
    rospy.init_node('test_handeye', anonymous=True)
    image_saver = MyImageSaver(cameraNS='camera')

    robot = init_robot_with_ik()
    hand_eye = MyHandEye("../config/hand_eye.npz")
    camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")

    mybag = MyBag(filename='follow_aruco.json')

    goal_pose = SE3(0,0,0.20) * SE3.Rx(-180, unit="deg")
    print('Goal Pose')
    goal_pose.printline()
    target_id = 4

    pose_list_size = 10
    pose_list = []

    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()

        # detect arucos in the frame
        poses_dict = get_aruco_poses(frame, 
                                    depth=None, 
                                    camera_intrinsics=camera_intrinsics, 
                                    default_marker_size=MARKER_SIZE, 
                                    draw_flag=True, 
                                    verbose=False)
        
        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
    
        if key == ord('q'):
            break
        
        if robot.is_moving():
            continue

        if target_id in poses_dict:
            pose = poses_dict[target_id] 
            pose_list.append(pose)

        if len(pose_list) >= pose_list_size:
            pose_mean = np.mean(pose_list, axis=0)
            pose_mean = pose_to_SE3(pose_mean)
            
            mybag.record("robot_pose", robot.get_pose())
            mybag.record("marker_pose", SE3_to_pose(pose_mean))

            pose_list = []
            camera_move = relative_pose(goal_pose, pose_mean, pattern="111001") 
            print("camera_move")
            camera_move.printline()
            gripper_action = hand_eye.gripper_move(camera_move)
            gripper_action = remove_small_motion(gripper_action)

            print("gripper_action")
            gripper_action.printline()
            robot.step_in_ee(action=gripper_action, wait=False)