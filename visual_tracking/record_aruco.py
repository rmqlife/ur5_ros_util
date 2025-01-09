import rospy
import cv2
import numpy as np
import os
from mySensor import MyImageSaver, load_intrinsics
from aruco_util import estimate_markers_poses, detect_aruco

from ik_step import lookup_action, key_map
from myPlanner import init_robot_with_ik

from myGlue import project_to_3d
from myPlanner import MyBag, SE3_to_pose
camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")
marker_size = 0.025

def get_aruco_poses(corners, ids, camera_intrinsics):
    # make sure the aruco's orientation in the camera view! 
    poses = estimate_markers_poses(corners, marker_size=marker_size, intrinsics=camera_intrinsics)  
    poses_dict = {}
    # detected
    if ids is not None:
        for k, iden in enumerate(ids):
            poses_dict[iden]=poses[k] 
    return poses_dict


def average_distance(pos_list):
    distances = []
    for i in range(len(pos_list)):
        a1 = pos_list[i]
        a2 = pos_list[(i + 1) % len(pos_list)]
        a1 = np.array(a1)
        a2 = np.array(a2)
        distance = np.linalg.norm(a1 - a2)
        distances.append(distance)
    mean_distance = np.mean(distances)
    return mean_distance

def measure_marker_size(corners, depth, camera_intrinsics):
    marker_sizes = []
    for corner in corners:
        points = project_to_3d(corner, depth, camera_intrinsics, show=False)
        mean_distance = average_distance(points)
        marker_sizes.append(mean_distance)
        
    mean_size = np.mean(marker_sizes)
    return mean_size

if __name__ == "__main__":
    # for eye in hand
    rospy.init_node('record_aruco')
    image_saver = MyImageSaver()
    mybag = MyBag(filename=os.path.join(image_saver.folder_path, 'record_aruco.json'))

    rospy.sleep(1)
    framedelay = 1000//20
    robot = init_robot_with_ik()

    robot_poses = []
    marker_poses = []# in frame of aruco marker
    home = image_saver.folder_path

    myid = 0
    while not rospy.is_shutdown():
        frame = image_saver.rgb_image
        depth = image_saver.depth_image
        corners, ids = detect_aruco(frame, draw_flag=True)

        poses_dict = get_aruco_poses(corners=corners, ids=ids, camera_intrinsics=camera_intrinsics)
        if myid in poses_dict:
            marker_pose = poses_dict[myid]
        else:
            marker_pose = None

        cv2.imshow('Camera', frame)
        # Exit on 'q' key press
        key = cv2.waitKey(framedelay) & 0xFF 
        if key == ord('q'):
            break

        elif key in key_map:
            code  = key_map[key]
            print(f"action {code}")
            action = lookup_action(code,  t_move=0.02, r_move=2)
            pose = robot.step(action=action, wait=False)

            # measure the marker size
            measured_marker_size = measure_marker_size(corners, depth, camera_intrinsics)
            print(f"Measured marker size: {measured_marker_size}")

        elif key == ord(' '):
            if marker_pose is not None:
                image_saver.record()
                se3_pose = robot.get_pose_se3()
                mybag.record("robot_pose", SE3_to_pose(se3_pose))
                mybag.record("marker_pose", marker_pose)
                print("recorded step")

    cv2.destroyAllWindows()
