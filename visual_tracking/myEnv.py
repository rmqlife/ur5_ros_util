from position_map import PositionMap
from mySensor import MyImageSaver, load_intrinsics
from myHandEye import MyHandEye
from myPlanner import init_robot_with_ik
from aruco_util import get_aruco_poses_info
import json
import numpy as np
import rospy
import cv2
import time
from position_map import same_position
import pickle
import os

def get_timestamp():
    date_time = time.strftime("%Y-%m-%d:%H-%M-%S", time.localtime())
    return date_time


class ArucoState:
    def __init__(self, robot_pose, marker_poses):
        self.robot_pose = robot_pose
        self.marker_poses = marker_poses
        self.timestamp = get_timestamp()

    def __gt__(self, other):
        return self.timestamp > other.timestamp

    def __eq__(self, other):
        if not same_position(self.robot_pose, other.robot_pose, t_threshold=0.02, rad_threshold=2*np.pi):
            print("robot pose not same")
            return False
        
        for marker_id in self.marker_poses:
            # print(f"marker {marker_id})")
            pose1 = self.marker_poses[marker_id]
            pose2 = other.marker_poses[marker_id]
            if not same_position(pose1, pose2):
                print(f"marker {marker_id} pose not same")
                return False
        return True
    
    

class MyStates:
    def __init__(self, filename):
        self.filename = filename
        self.states = []
        if os.path.exists(filename):
            with open(filename, "rb") as f:
                self.states = pickle.load(f)

    def __getitem__(self, index):
        if index < 0 or index >= len(self.states):
            return None
        return self.states[index]
    
    def empty(self):
        return len(self.states) == 0

    def add_state(self, state):
        self.states.append(state)

    def save(self):
        with open(self.filename, "wb") as f:
            pickle.dump(self.states, f)
    
    def index(self, state):
        return self.states.index(state)
    
    # last state
    def last_state(self):
        if len(self.states) == 0:
            return None
        return self.states[-1]
    
    def push_back(self, state):
        if len(self.states) > 0:
            if self.last_state()==state:
                return -1
        print("adding new state")
        self.states.append(state)
        return len(self.states)-1



class MyEnv:
    def __init__(self, states_path):
        rospy.init_node('my_env_node')

        self.robot = init_robot_with_ik()
        self.image_saver = MyImageSaver(cameraNS='camera')

        self.hand_eye = MyHandEye("../config/hand_eye.json")

        with open("../config/camera_intrinsics.json", 'r') as f:
            self.camera_intrinsics = json.load(f)

        with open('../config/marker_info.json', 'r') as f:
            self.marker_info = json.load(f)

        self.id_list = [self.marker_info[key]["marker_id"] for key in self.marker_info]
        self.position_map = PositionMap(id_list=self.id_list, camera_num=1)

        # states organizer
        self.states = MyStates(states_path)

    
    def get_state(self):
        frame, _ = self.image_saver.get_frames()
        frame_draw = frame.copy()
        # 获取marker的pose
        marker_poses = get_aruco_poses_info(frame_draw, self.camera_intrinsics, self.marker_info, verbose=False)
        for marker_id in marker_poses:
            marker_pose = marker_poses[marker_id]
            marker_pose = self.robot.get_pose_se3() * self.hand_eye.T_c2g * marker_pose
            # 更新位置
            self.position_map.update_position(marker_id, marker_pose, verbose=True)

        state = ArucoState(robot_pose=self.robot.get_pose_se3(),
                           marker_poses=self.position_map.position_map.copy())
        return state, frame_draw
           
if __name__ == "__main__":
    env = MyEnv("./data/move_ref.pkl")

    recording = False
    while True:
        state, frame_draw = env.get_state()
        env.states.push_back(state)

        cv2.imshow("camera", frame_draw)
        key = cv2.waitKey(1000//60) & 0xFF 
            
        if key == ord('q'):
            env.states.save()
            print(f"\n\n saving {len(env.states.states)} states")
            break

        elif key == ord('p'):
            print(env.position_map)

        if key == ord('s'):
            # saving states
            env.states.save()
            print(f"\n\n saving {len(env.states.states)} states")
            