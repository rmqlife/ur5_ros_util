from myGlue import MyGlue
from myApp.ik_step import *
from mySensor import MyImageSaver,  load_intrinsics
from myPlanner import init_robot_with_ik
import os
from myHandEye import MyHandEye
from myStates import MyStates, load_state, save_state

framedelay = 1000//20

def filter_action(action, t_thresh=0.03, rad_thresh=0.05):
    from follow_aruco import se3_to_euler6d, euler_to_se3
    
    action = se3_to_euler6d(action)
    t = np.array(action[:3])
    rad = np.array(action[3:])
    print("t", t)
    print("rad", rad)
    t_norm = np.linalg.norm(t)
    rad_norm = np.linalg.norm(rad)
    if t_norm > t_thresh:
        t = t_thresh / t_norm * t
        rad = np.zeros(3) #[0,0,0]
        print("new t", t)
    elif rad_norm > rad_thresh:
        t = np.zeros(3)
        rad = rad_thresh / rad_norm * rad
        print('now rad', rad)
    # euler = 1*np.array(euler)
    action = t.tolist() + rad.tolist()
    action = euler_to_se3(action)
    return action


if __name__=="__main__":
    save_data_dir = "data_states/recorded_0206/"


    rospy.init_node('record_visual_states')
    rospy.sleep(1)
    image_saver = MyImageSaver(cameraNS="camera")
    robot = init_robot_with_ik()


    glue = MyGlue(match_type="Aruco") # Aruco LightGlue
    intrinsics = load_intrinsics('../config/camera_intrinsics.json')
    hand_eye = MyHandEye("../config/hand_eye.npz")


    states = MyStates(save_data_dir)
    states.sync_to_action_queue()
    goal = states.popfront()


    while not rospy.is_shutdown():
        frame = image_saver.rgb_image
        show_frame = frame.copy() # in case of frame is ruined by painting frame
        depth_frame = image_saver.depth_image

        if goal is None:
            goal = dict()
            goal["frame"] = frame.copy()
            goal["depth_frame"] = depth_frame.copy()

        src_pts, dst_pts = glue.match(frame, goal["frame"])
        src_pts = [pt.tolist() for pt in src_pts] 
        dst_pts = [pt.tolist() for pt in dst_pts] 
        for (x1, y1), (x2, y2) in zip(src_pts, dst_pts):
            cv2.line(show_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)

        cv2.imshow('Camera', show_frame)
        # Exit on 'q' key press
        key = cv2.waitKey(framedelay) & 0xFF 
        if key == ord('q'):
            break

        elif key in key_map:
            code = key_map[key]
            print(f"Action {code}")
            action = lookup_action(code)
            action = hand_eye.gripper_move(action)
            robot.step_in_ee(action=action, wait=False)

        elif key == ord('m'):      
            if goal is None:
                print("loaded a None state")
                continue
            src_pts_3d, dst_pts_3d = glue.map_3d_pts(src_pts, dst_pts, depth_frame, goal["depth_frame"], intrinsics)
            print("src_pts_3d", src_pts_3d.shape)
            print(src_pts_3d)
            
            print("dst_pts_3d")
            print(dst_pts_3d)
            # for lightglue
            R, t = compute_rigid_transform(dst_pts_3d, src_pts_3d, enable_R=True)
            camera_move = Rt_to_SE3(R, t)
            camera_move.printline()
            gripper_move = hand_eye.gripper_move(camera_move)

            # filter action to move step by step
            gripper_move = filter_action(gripper_move)
            robot.step_in_ee(action=gripper_move, wait=False)

        elif key == ord('t'):
            print("toggle goal")
            if states.action_queue.empty():
                print("reloading states")
                ret = states.sync_to_action_queue()
                if ret.empty():
                    print("no states to read")
                    continue
            goal = states.popfront()

        elif key == ord('g'):
            # setup goal
            goal = dict()
            goal["frame"] = frame.copy()
            goal["depth_frame"] = depth_frame.copy()
            goal["joints"] = robot.get_joints()
            goal["pose"] = robot.get_pose()
            filename = states.save_state(goal)
            filename = filename.replace(".npz", ".jpg")
            cv2.imwrite(filename, goal["frame"])
            
    cv2.destroyAllWindows()