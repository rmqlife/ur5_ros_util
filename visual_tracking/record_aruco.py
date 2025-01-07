from aruco_util import *
from mySensor import MyImageSaver
import rospy
import cv2
# from replay_aruco_poses import *
from std_msgs.msg import Float32
from ik_step import *
import os

def get_aruco_poses(corners, ids, intrinsics):
    # make sure the aruco's orientation in the camera view! 
    poses = estimate_markers_poses(corners, marker_size=0.10, intrinsics=intrinsics)  # Marker size in meters
    poses_dict = {}
    # detected
    if ids is not None:
        for k, iden in enumerate(ids):
            poses_dict[iden]=poses[k] 
    return poses_dict


def get_cam_pose(frame, intrinsics):
    print("intrinsics", intrinsics)

    corners, ids = detect_aruco(frame, draw_flag=True)# 
    print('get cam ids',ids)
    poses_dict = get_aruco_poses(corners=corners, ids=ids, intrinsics=intrinsics)
    id = 0
    current_cam = None
    if id in poses_dict:
        current_pose = poses_dict[id]
            # compute the R, t
        current_cam = inverse_pose(current_pose)
            # compute 
        # print('cam', np.round(current_cam[:3], 3))
    return current_cam

def get_marker_pose(frame, id=0, draw=True, intrinsics = intrinsics_d435):
    corners, ids = detect_aruco(frame, draw_flag=draw)# 
    if ids is not None and len(ids)>0:
        poses_dict = get_aruco_poses(corners=corners, ids=ids, intrinsics=intrinsics)
        for i, c in zip(ids, corners):
            if i == id:
                # exchange x, y axis
                # transform = SE3([[0,1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
                pose = pose_to_SE3(poses_dict[id])
                # pose = transform * pose
                
                return pose, c
    
    return None, None



def record_in_hand():
    # for eye in hand
    rospy.init_node('follow_aruco')
    image_saver = MyImageSaver(namespace="camera1")
    rospy.sleep(1)
    framedelay = 1000//20
    # intrinsics = load_intrinsics("slam_data/intrinsics_d435.json")
    robot = init_robot("robot1")
    robot_without_basetransformation = MyIK()
    goal_pose = None
    goal_corner = None
    robot_poses = []
    marker_poses = []# in frame of aruco marker
    home = image_saver.folder_path

    while not rospy.is_shutdown():
        frame = image_saver.rgb_image
        
        marker_pose, corner = get_marker_pose(frame, 0)
        
        if goal_corner is not None and corner is not None:
            for (x1, y1), (x2, y2) in zip(corner, goal_corner):
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)

        cv2.imshow('Camera', frame)
        # Exit on 'q' key press
        key = cv2.waitKey(framedelay) & 0xFF 
        if key == ord('q'):

            break
        if key == ord('s'):
            if marker_pose != None:
                joints = robot.get_joints()
                robot_pose = robot_without_basetransformation.fk(joints)
                robot_poses.append(robot_pose)
                marker_poses.append(SE3_to_pose(marker_pose))
                image_saver.record()
            else:
                print("cannot detect id 0")


        elif key in key_map:
            code  = key_map[key]
            print(f"action {code}")
            action = lookup_action(code)
            pose = robot.step(action=action, wait=False)
            print('robot pose', np.round(pose[:3], 3))

        if key == ord('m') and goal_pose is not None and marker_pose is not None:
            move = goal_pose * marker_pose.inv()
            move = SE3(goal_pose.t) * SE3(marker_pose.t).inv()
            print('movement')
            move.printline()
            robot.step(action=move, wait=False)

        if key == ord('g'):
            # setup goal
            if marker_pose is not None:
                goal_pose = marker_pose.copy()
                goal_corner = corner.copy()
                print('set goal as')
                goal_pose.printline()
            else:
                print('no valid marker pose')
    np.save(os.path.join(home,  'robot_poses'), robot_poses)
    np.save(os.path.join(home,  'marker_poses'), marker_poses)
    cv2.destroyAllWindows()

def record_to_hand():
    rospy.init_node('follow_aruco')
    image_saver = MyImageSaver(namespace="camera1")
    image_saver2 = MyImageSaver(namespace="camera3")
    rospy.sleep(1)
    framedelay = 1000//20
    # intrinsics = load_intrinsics("slam_data/intrinsics_d435.json")
    robot = init_robot("robot1")
    robot_without_basetransformation = MyIK()
    goal_pose = None
    goal_corner = None
    robot_poses = []
    marker_poses = []# in frame of aruco marker
    marker_poses2 = []
    home = image_saver.folder_path

    while not rospy.is_shutdown():
        frame = image_saver.rgb_image   
        frame2 = image_saver2.rgb_image


        
        marker_pose, corner = get_marker_pose(frame, 0, intrinsics=intrinsics_d435)
        marker_pose2, corner2 = get_marker_pose(frame2, 0, intrinsics=intrinsics_d435)
        
        if goal_corner is not None and corner is not None:
            for (x1, y1), (x2, y2) in zip(corner, goal_corner):
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)

        if goal_corner is not None and corner2 is not None:
            for (x1, y1), (x2, y2) in zip(corner2, goal_corner):
                cv2.line(frame2, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)

        cv2.imshow('Camera', frame)
        cv2.imshow('Camera2', frame2)

        # Exit on 'q' key press
        key = cv2.waitKey(framedelay) & 0xFF 
        if key == ord('q'):

            break
        if key == ord('s'):
            if marker_pose != None:
                joints = robot.get_joints()
                robot_pose = robot_without_basetransformation.fk(joints)
                robot_poses.append(robot_pose)
                marker_poses.append(SE3_to_pose(marker_pose))
                image_saver.record()
            else:
                print("cannot detect id 0")
            if marker_pose2 != None and len(marker_poses2)==0:
                marker_poses2.append(SE3_to_pose(marker_pose2))
                image_saver2.record()


        elif key in key_map:
            code  = key_map[key]
            print(f"action {code}")
            action = lookup_action(code)
            pose = robot.step(action=action, wait=False)
            print('robot pose', np.round(pose[:3], 3))

        if key == ord('m') and goal_pose is not None and marker_pose is not None:
            move = goal_pose * marker_pose.inv()
            move = SE3(goal_pose.t) * SE3(marker_pose.t).inv()
            print('movement')
            move.printline()
            robot.step(action=move, wait=False)

        if key == ord('g'):
            # setup goal
            if marker_pose is not None:
                goal_pose = marker_pose.copy()
                goal_corner = corner.copy()
                print('set goal as')
                goal_pose.printline()
            else:
                print('no valid marker pose')
    np.save(os.path.join(home,  'robot_poses'), robot_poses)
    np.save(os.path.join(home,  'marker_poses'), marker_poses)
    np.save(os.path.join(home,  'marker_poses2'), marker_poses2)
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    record_in_hand()