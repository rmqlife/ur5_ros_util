import rospy
import cv2
import numpy as np
import os
from mySensor import MyImageSaver, load_intrinsics
from aruco_util import get_aruco_poses

from myApp.ik_step import lookup_action, key_map
from myPlanner import init_robot_with_ik
from myPlanner import MyBag, SE3_to_pose



camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")

MARKER_SIZE = 0.0258

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
        frame_copy = frame.copy()
        depth = image_saver.depth_image
        # filter corners and ids by desired_ids
        poses_dict = get_aruco_poses(frame=frame, 
                                     depth=depth, 
                                     camera_intrinsics=camera_intrinsics, 
                                     default_marker_size=MARKER_SIZE, 
                                     draw_flag=True, 
                                     verbose=False)

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
    
        elif key == ord(' '):
            if marker_pose is not None:
                image_saver.record()
                se3_pose = robot.get_pose_se3()
                mybag.record("robot_pose", SE3_to_pose(se3_pose))
                mybag.record("marker_pose", marker_pose)
                print("recorded step")
        
        elif key == ord('v'):
            poses_dict = get_aruco_poses(frame_copy, depth, 
                                         camera_intrinsics=camera_intrinsics, 
                                         default_marker_size=MARKER_SIZE, 
                                         draw_flag=True, 
                                         verbose=True)
            # if 0, 1, 2 are in poses_dict, print 3d distance between 0 and 1, 1 and 2
            if all(i in poses_dict for i in [0, 1, 2, 3]):
                for i, j in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                    distance = np.linalg.norm(poses_dict[i][:3] - poses_dict[j][:3])
                    print(f"distance between {i} and {j}: {distance}")

    cv2.destroyAllWindows()