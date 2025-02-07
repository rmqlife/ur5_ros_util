import numpy as np
from spatialmath import SE3
from myPlanner import init_robot_with_ik, MyBag
from myPlanner.pose_util import *
from mySensor import MyImageSaver
from myApp import key_map, lookup_action
import rospy
import cv2
from aruco_util import *
from mySensor import load_intrinsics
from myHandEye import MyHandEye
import os
from follow_aruco import relative_pose, MARKER_SIZE, framedelay, se3_to_euler6d, euler_to_se3
from myStates import MyStates, load_state, save_state

def corners_to_poses(corners, camera_intrinsics,  default_marker_size,  verbose=False):
    poses_list = []
    # make sure the aruco's orientation in the camera view! 
    marker_size = default_marker_size
    for corner in corners:
        pose = estimate_marker_pose(corner, marker_size=marker_size, intrinsics=camera_intrinsics) 
        poses_list.append(pose)
    return poses_list

class ArucoState:
    def __init__(self, ids, pts, corners):
        self.ids = ids
        self.pts = {id: pt for id, pt in zip(ids, pts)}
        self.corners = {id: corner for id, corner in zip(ids, corners)}
        self.data = dict()
        self.data['ids'] = ids
        self.data['pts'] = pts
        self.data['corners'] = corners

    def pt(self, id):
        return self.pts[id]
    
    def corner(self, id):
        return self.corners[id]



def mean_pose_in_SE3(poses):
    poses = [se3_to_euler6d(pose) for pose in poses]
    mean_pose = np.mean(poses, axis=0)
    mean_pose = euler_to_se3(mean_pose)
    return mean_pose


def match_aruco_state(state0, state1):
    ids0 = state0.ids
    ids1 = state1.ids

    common_ids = set(ids0) & set(ids1)
    common_ids = list(common_ids)
    pts0 = []
    pts1 = []
    if common_ids:
        # Extract pots for common IDs
        for id in common_ids:
            pts0 += state0.pt(id)
            pts1 += state1.pt(id)
        # filter out zeros points
        from myGlue import filter_out_zeros_points
        pts0, pts1 = filter_out_zeros_points(pts0, pts1)
        
        R, t = find_transformation(pts0, pts1)
        action = Rt_to_SE3(R, t)
        return action
    return None


def corners_to_pts(corners):
    pts = np.array(corners).reshape(-1, 2)
    pts = [pt.tolist() for pt in pts] 
    return pts

def draw_match_line(frame, state0, state1):
    ids0 = state0.ids
    ids1 = state1.ids

    common_ids = set(ids0) & set(ids1)
    common_ids = list(common_ids)
    if common_ids:
        # Extract poses for common IDs
        corners0 = [state0.corner(id) for id in common_ids]
        corners1 = [state1.corner(id) for id in common_ids]

        # reshape to 2d array
        corners0 = corners_to_pts(corners0)   
        corners1 = corners_to_pts(corners1)

        for (x1, y1), (x2, y2) in zip(corners0, corners1):
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)


def project_corners(corners, depth, intrinsics):
    corners = np.array(corners).reshape(-1, 2)
    pts = project_to_3d(corners, depth, intrinsics)
    pts = np.array(pts)
    pts = pts.reshape(-1,4,3)
    pts = [pt.tolist() for pt in pts]
    return pts


if __name__ == "__main__":
    rospy.init_node('pick_aruco', anonymous=True)
    save_data_dir = "data_states/recorded_arucos_0207/"
    if not os.path.exists(save_data_dir):
        os.makedirs(save_data_dir)
    states_path = os.path.join(save_data_dir, "states.pkl")

    image_saver = MyImageSaver(cameraNS='camera')

    robot = init_robot_with_ik()
    hand_eye = MyHandEye("../config/hand_eye.npz")
    camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")

    # load pkl from save_data_dir
    import pickle
    goal_state = None
    if os.path.exists(states_path):
        with open(states_path, "rb") as f:
            state = pickle.load(f)
            goal_state = state["aruco_state"]
    
    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()

        # detect arucos in the frame
        aruco_corners, aruco_ids = detect_aruco(frame, draw_flag=True)
        
        aruco_pts = project_corners(aruco_corners, depth, camera_intrinsics)
        aruco_state = ArucoState(aruco_ids, aruco_pts, aruco_corners)        

        if goal_state is not None:
            draw_match_line(frame, aruco_state, goal_state)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
    
        if key == ord('q'):
            break
        elif key in key_map:
            code = key_map[key]
            print(f"Action {code}")
            action = lookup_action(code)
            action = hand_eye.gripper_move(action)
            robot.step_in_ee(action=action, wait=False)

        # building the arucos's poses
        elif key == ord('b'):
            print(aruco_state.ids)
            
        elif key == ord('m') and goal_state is not None:
            camera_move = match_aruco_state(goal_state, aruco_state)
            if camera_move is None:
                print("no match")
                continue
            print("camera_move")
            camera_move.printline()
            gripper_action = hand_eye.gripper_move(camera_move)
            print("gripper_action")
            gripper_action.printline()
            robot.step_in_ee(action=gripper_action, wait=False)

        elif key == ord('g'):
            # setup goal
            goal_state = aruco_state
            
            state = dict()
            state["robot_pose"] = robot.get_pose()
            state["aruco_state"] = goal_state
            import time
            state["timestamp"] = time.strftime("%Y%m%d-%H%M%S")
            print(state["timestamp"])

            # dump state to pkl
            import pickle
            with open(states_path, "wb") as f:
                pickle.dump(state, f)
                
            cv2.imwrite(os.path.join(save_data_dir, f"{state['timestamp']}.jpg"), frame)

            
