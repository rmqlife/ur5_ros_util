import cv2
import numpy as np
import json
from spatialmath import SE3
from spatialmath.base import trnorm
import numpy as np

from my_kalmen_filter import KalmenFilter
import rospy
from myPlanner import init_robot_with_ik, se3_to_euler6d
from mySensor import MyImageSaver, load_intrinsics
from myHandEye import MyHandEye
from aruco_util import get_aruco_poses_info     
# func 分析frame 
'''
    1. 两个对象
        1.1 一个对象是marker
        1.2 一个对象是camera
'''

class Camera2World:
    def __init__(self,camera_name):
        """
        Initialize Camera2World transformer.
        
        Args:
            camera_name (str): Name identifier for the camera
        """
        self.camera_name = camera_name
        self.camera_pose = None

    def set_camera_pose(self,camera_pose):
        self.camera_pose = camera_pose
    
    def convert_camera_pose_to_world_pose(self,marker_pose):
        if self.camera_pose is None:
            raise ValueError("Camera pose must be set before converting coordinates")
        return self.camera_pose.inv() * marker_pose


# 记录更新位置
'''
    Map: 
        key: marker id
        value: marker pose
'''


def same_position(pose1, pose2, rad_threshold=5, t_threshold=0.02):
    if pose1 is None or pose2 is None:
        return False
    
    euler1 = np.array(se3_to_euler6d(pose1))
    euler2 = np.array(se3_to_euler6d(pose2))
    
    rad_diff = np.linalg.norm(euler1[3:] - euler2[3:])
    t_diff = np.linalg.norm(euler1[:3] - euler2[:3])
    
    return rad_diff < rad_threshold and t_diff < t_threshold


class PositionMap:
    def __init__(self, id_list, camera_num):
        """
        Initialize position tracking for multiple markers.
        
        Args:
            id_list: List of marker IDs to track
        """
        self.position_map = {}
        self.overall_map = []
        
        self.camera_num = camera_num
        self.filter_list = {id: KalmenFilter() for id in id_list}
        self.temp_map = {id: [] for id in id_list}

    def reset_temp_map(self,marker_id):
        self.temp_map[marker_id] = []

    def filter_pose(self, marker_id, marker_pose):
        """Apply Kalman filter to marker pose."""
        if marker_id not in self.filter_list:
            self.filter_list[marker_id] = KalmenFilter()
            return marker_pose
            
        self.filter_list[marker_id].new_markerpose(marker_pose)
        self.filter_list[marker_id].Kalman_Filter()
        return self.filter_list[marker_id].get_pose()

    def update_position(self, marker_id, marker_pose, verbose=False):
        """Update marker position if significantly different from previous position."""        
        if marker_pose is None:
            # 如果marker_pose为None，则将该marker_id的位置设置为None
            # self.position_map[marker_id] = None
            self.temp_map[marker_id].append(None)
            return

        filtered_pose = self.filter_pose(marker_id, marker_pose)
        
        if (marker_id in self.position_map and 
            self.position_map[marker_id] is not None and
            same_position(self.position_map[marker_id], filtered_pose)):
            # 如果marker_pose与之前的位置相同，则不更新位置
            return

        # self.temp_map[marker_id].append(filtered_pose)
        if verbose:
            print(f"Updated position for marker {marker_id}:")
            filtered_pose.printline()

        self.position_map[marker_id] = filtered_pose

    def get_position(self, marker_id):
        return self.position_map[marker_id]
    
    def add2overall_map(self):
        self.overall_map.append(self.position_map.copy())


def main():
    with open('marker_info.json', 'r') as f:
        marker_info = json.load(f)
    print("marker_info:", marker_info)

    rospy.init_node('new_follow_aruco')
    image_saver = MyImageSaver(cameraNS='camera')
    robot = init_robot_with_ik()
    hand_eye = MyHandEye("../config/hand_eye.npz")
    camera_intrinsics = load_intrinsics()

    id_list = [marker_info[key]["marker_id"] for key in marker_info]
    print("id_list", id_list)
    position_map = PositionMap(id_list=id_list, camera_num=1)

    framedelay = 1000//60
    while True:
        frame, depth = image_saver.get_frames()          
        img_copy = frame.copy()

        # 获取marker的pose
        marker_poses = get_aruco_poses_info(img_copy, camera_intrinsics, marker_info, verbose=False)
        marker_map = {}
        for marker_id in marker_poses:
            marker_pose = marker_poses[marker_id]
            marker_pose = robot.get_pose_se3() * hand_eye.T_c2g * marker_pose
            marker_map[marker_id] = marker_pose
            # 更新位置
            position_map.update_position(marker_id, marker_pose, verbose=True)
        # Display the image
        position_map.add2overall_map()
        cv2.imshow("camera", img_copy)
        
        key = cv2.waitKey(framedelay) & 0xFF 

        if key == ord('q'):
            break

        elif key == ord('p'):
            print("marker_poses:")
            for marker_id in marker_map:
                print(f"marker_id: {marker_id}")
                marker_pose = marker_map[marker_id]
                marker_pose.printline()
            print("position_map:")
            print(position_map.position_map)


if __name__ == "__main__":
    main()
    
