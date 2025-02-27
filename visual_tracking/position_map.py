import cv2
import numpy as np
import json
from spatialmath import SE3
from spatialmath.base import trnorm
import numpy as np
from kalman_filter import KalmenFilter
import rospy
from myPlanner import init_robot_with_ik, se3_to_str
from mySensor import MyImageSaver, load_intrinsics
from myHandEye import MyHandEye
from aruco_util import get_aruco_poses_info     

def same_position(pose1, pose2,  t_threshold=0.02, rad_threshold=0.2):
    """
    Compare two poses to determine if they are effectively the same position.
    
    Args:
        pose1, pose2: Marker poses to compare
        angle_threshold: Maximum angle difference in radians (default: 0.1)
        translation_threshold: Maximum translation difference in meters (default: 0.01)
    
    Returns:
        bool: True if poses are considered same, False otherwise
    """
    if pose1 is None and pose2 is None:
        return True
    if pose1 is None or pose2 is None:
        return False
    R_diff = np.dot(pose1.R.T, pose2.R)
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    translation_norm = np.linalg.norm(pose1.t - pose2.t)
    return angle < rad_threshold and translation_norm < t_threshold


class PositionMap:
    def __init__(self, id_list, camera_num):
        """
        Initialize position tracking for multiple markers.
        
        Args:
            id_list: List of marker IDs to track
        """
        self.marker_poses = {}
        self.camera_num = camera_num
        self.filter_list = {id: KalmenFilter() for id in id_list}

    def filter_pose(self, marker_id, marker_pose):
        """Apply Kalman filter to marker pose."""
        if marker_id not in self.filter_list:
            self.filter_list[marker_id] = KalmenFilter()
            return marker_pose
            
        self.filter_list[marker_id].new_markerpose(marker_pose)
        self.filter_list[marker_id].Kalman_Filter()
        return self.filter_list[marker_id].get_pose()

    # what if the marker_pose is None? Partial observation problem
    def update_position(self, marker_id, marker_pose, verbose=False):
        """Update marker position if significantly different from previous position."""        
        filtered_pose = self.filter_pose(marker_id, marker_pose)
        # if not changed position, return -1
        if (marker_id in self.marker_poses and 
            self.marker_poses[marker_id] is not None and
            same_position(self.marker_poses[marker_id], filtered_pose)):
            return -1
        else:
            self.marker_poses[marker_id] = filtered_pose
            if verbose:
                print(f"Updated position for marker {marker_id}:")
                filtered_pose.printline()
            return 1

    def get_position(self, marker_id):
        return self.marker_poses[marker_id]
    
    def __str__(self):
        """Return a string representation of the current positions of all tracked markers."""
        position_str = "Position Map:\n"
        for marker_id, pose in self.marker_poses.items():
            position_str += f"ID: {marker_id} \n {se3_to_str(pose)}\n"
        return position_str

    def printline(self):
        """Print the current positions of all tracked markers."""
        print(self.__str__())

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
        frame, _ = image_saver.get_frames()          
        img_copy = frame.copy()

        # 获取marker的pose
        marker_poses = get_aruco_poses_info(img_copy, camera_intrinsics, marker_info, verbose=False)
        for marker_id in marker_poses:
            marker_pose = marker_poses[marker_id]
            marker_pose = robot.get_pose_se3() * hand_eye.T_c2g * marker_pose
            # 更新位置
            position_map.update_position(marker_id, marker_pose, verbose=True)
        # Display the image
        cv2.imshow("camera", img_copy)
        
        key = cv2.waitKey(framedelay) & 0xFF 

        if key == ord('q'):
            break

        elif key == ord('p'):
            position_map.printline()


if __name__ == "__main__":
    main()
    
