from mySensor import MyImageSaver, load_intrinsics
from myPlanner import init_robot_with_ik, SE3_to_pose, pose_to_SE3, se3_to_str, same_position
from visual_tracking.position_map import PositionMap
from visual_tracking.aruco_util import get_aruco_poses_info
from visual_tracking.myHandEye import MyHandEye

import json
import numpy as np
import rospy
import cv2
import time
import pickle
import os
from std_msgs.msg import String, Float64MultiArray

ARUCO_STATE_TOPIC = "/aruco_state"

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
        if not same_position(self.robot_pose, other.robot_pose, t_threshold=0.01, rad_threshold=2*np.pi):
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
    
    def __str__(self):
        return f"robot pose: {se3_to_str(self.robot_pose)}\n marker poses: {self.marker_poses}"

class MyStates:
    def __init__(self, filename=None):
        self.empty()
        if filename is not None:
            print(f"loading states from {filename}")
            self.load(filename)
            
    def load(self, filename):
        if os.path.exists(filename):
            with open(filename, "rb") as f:
                self.states = pickle.load(f)
                print(f"loaded {len(self.states)} states")
        else:
            print(f"file {filename} not found")

    def get_states(self):
        return self.states

    def __getitem__(self, index):
        if index < 0 or index >= len(self.states):
            return None
        return self.states[index]
    
    def is_empty(self):
        return len(self.states) == 0
    
    def empty(self):
        self.states = []

    def save(self, filename):
        with open(filename, "wb") as f:
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
    def __init__(self, is_publisher):
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
        self.is_publisher = is_publisher
        if self.is_publisher:
            self.aruco_state_pub = rospy.Publisher(ARUCO_STATE_TOPIC, Float64MultiArray, queue_size=10)
            # Add subscriber
        else:
            self.aruco_state_sub = rospy.Subscriber(ARUCO_STATE_TOPIC, Float64MultiArray, self.subscribe_state_callback)
            self.latest_received_state = None
    
    def get_state(self):
        frame, _ = self.image_saver.get_frames()
        frame_draw = frame.copy()
        if self.is_publisher:
            # 获取marker的pose
            marker_poses = get_aruco_poses_info(frame_draw, self.camera_intrinsics, self.marker_info, verbose=False)
            for marker_id in marker_poses:
                marker_pose = marker_poses[marker_id]
                marker_pose = self.robot.get_pose() * self.hand_eye.T_c2g * marker_pose
                # 更新位置
                self.position_map.update_position(marker_id, marker_pose, verbose=True)

            state = ArucoState(robot_pose=self.robot.get_pose(),
                            marker_poses=self.position_map.marker_poses.copy())
            self.publish_state(state)
        else: # subscribe mode
            state = self.latest_received_state
        return state, frame_draw
    
    # publish ArucoState to ros
    def publish_state(self, state):
        """
        Publishes marker poses in a single Float64MultiArray message.
        Format: [num_markers, (marker_id1, x1, y1, z1, qx1, qy1, qz1, qw1), (marker_id2, ...), ...]
        """
        marker_poses = state.marker_poses.copy()
        
        # Create array to hold all data
        # First element is number of markers
        data = [0]
        # Add each marker's data (id + pose)
        for marker_id in marker_poses:
            marker_pose = marker_poses[marker_id]
            pose_array = SE3_to_pose(marker_pose)  # Convert to [x, y, z, qx, qy, qz, qw]
            
            # Add marker_id followed by its pose
            data.extend([float(marker_id)])  # Marker ID
            data.extend(pose_array)  # Pose (7 elements)
        
        # Create and publish the message
        msg = Float64MultiArray()
        msg.data = data
        self.aruco_state_pub.publish(msg)

    def subscribe_state_callback(self, msg):
        """
        Callback function for marker poses subscription.
        Format: [num_markers, (marker_id1, x1, y1, z1, qx1, qy1, qz1, qw1), (marker_id2, ...), ...]
        Updates latest_received_state with the received data.
        """
        try:
            data = msg.data
            marker_poses = {}
            # Parse the data array
            idx = 1  # Start after num_markers
            while idx < len(data):
                marker_id = int(data[idx])
                pose_array = data[idx + 1:idx + 8]  # Get next 7 values for pose
                
                # Convert pose array back to SE3
                marker_pose = pose_to_SE3(pose_array)
                
                marker_poses[marker_id] = marker_pose
                idx += 8  # Move to next marker (1 for ID + 7 for pose)
            
            # Create and store ArucoState object
            self.latest_received_state = ArucoState(
                robot_pose=self.robot.get_pose(),  # Current robot pose
                marker_poses=marker_poses
            )
            
        except Exception as e:
            rospy.logwarn(f"Failed to process marker poses message: {e}")
    
if __name__ == "__main__":
    rospy.init_node('my_env_node')
    env = MyEnv(is_publisher=True)
    while True:
        state, frame_draw = env.get_state()
        cv2.imshow("camera", frame_draw)
        key = cv2.waitKey(1000//60) & 0xFF 
        if key == ord('q'):
            break