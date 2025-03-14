from myGlue import MyGlue
from myApp.ik_step import *
from mySensor import MyImageSaver,  load_intrinsics
from myPlanner import init_robot_with_ik
import os
from myHandEye import MyHandEye
from myStates import load_state, save_state
data_dir = "pick_data"



if __name__=="__main__":
    rospy.init_node('pick_glue')
    image_saver = MyImageSaver()
    rospy.sleep(1)
    framedelay = 1000//20

    robot = init_robot_with_ik()

    glue = MyGlue(match_type="Aruco") # Aruco LightGlue
    goal = load_state(filename="goal_state.npz")

    intrinsics = load_intrinsics('../config/camera_intrinsics.json')
    hand_eye = MyHandEye("../config/hand_eye.npz")


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
            action.printline()
            action = hand_eye.gripper_move(action)
            robot.step_in_ee(action=action, wait=False)

        elif key == ord('m'):      

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
            robot.step_in_ee(action=gripper_move, wait=False)
            
        elif key == ord('g'):
            # setup goal
            goal = dict()
            goal["frame"] = frame.copy()
            goal["depth_frame"] = depth_frame.copy()
            goal["joints"] = robot.get_joints()
            goal["pose"] = robot.get_pose()
            save_state(goal, filename="goal_state.npz")
            print('set goal')
            
    cv2.destroyAllWindows()