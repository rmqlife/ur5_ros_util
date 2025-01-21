import numpy as np
from spatialmath import SE3
from myPlanner import init_robot_with_ik
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

if __name__ == "__main__":
    rospy.init_node('test_handeye', anonymous=True)
    image_saver = MyImageSaver(cameraNS='camera')
    robot = init_robot_with_ik()
    hand_eye = MyHandEye("../config/hand_eye.npz")
    camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")

    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()

        # detect arucos in the frame
        poses_dict = get_aruco_poses(frame, 
                                    depth, 
                                    camera_intrinsics=camera_intrinsics, 
                                    default_marker_size=MARKER_SIZE, 
                                    draw_flag=True, 
                                    verbose=False)
        
        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
    
        if key == ord('q'):
            break
        elif key in key_map:
            code  = key_map[key]
            print(f"action {code}")
            action = lookup_action(code)

            print("action in robot frame")
            action.printline()
            action = hand_eye.gripper_move(action)
            print("action in camera frame")
            action.printline()

            robot.step_in_ee(action=action, wait=False)

        # measure distance between arucos
        elif key == ord('m'):
            if all(i in poses_dict for i in [0, 1, 2, 3]):
                for i, j in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                    distance = np.linalg.norm(poses_dict[i][:3] - poses_dict[j][:3])
                    print(f"Distance between {i} and {j}: {distance}")
        
