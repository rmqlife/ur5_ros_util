from myPlanner import init_robot_with_ik
from myPlanner.pose_util import *
from mySensor import MyImageSaver
from myApp import key_map, lookup_action
import rospy
import cv2
from aruco_util import *
from mySensor import load_intrinsics
from myHandEye import MyHandEye
import os
from follow_aruco import framedelay 
from myArucoStates import  MyArucoStates, match_aruco_state, draw_match_line, build_aruco_state

def se3_norm(se3, t_only=True):
    if t_only:
        return np.linalg.norm(se3.t)
    else:
        euler = se3_to_euler6d(se3)
        return np.linalg.norm(euler[:3]) + np.linalg.norm(euler[3:])

if __name__ == "__main__":
    rospy.init_node('pick_aruco', anonymous=True)
    save_data_dir = "data_states/test_goals/"
    if not os.path.exists(save_data_dir):
        os.makedirs(save_data_dir)

    image_saver = MyImageSaver(cameraNS='camera')

    robot = init_robot_with_ik()
    hand_eye = MyHandEye("../config/hand_eye.npz")
    camera_intrinsics = load_intrinsics("../config/camera_intrinsics.json")

    # load pkl from save_data_dir
    goal_states = MyArucoStates(os.path.join(save_data_dir, "goal_states.pkl"))
    goal_state = goal_states.next_state(None)

    matched_states = MyArucoStates(os.path.join(save_data_dir, "matched_states.pkl"))
    if matched_states.empty():
        print("init matched states from goal states")
        for goal in goal_states.states:
            matched_states.add_state(goal)

    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()

        # detect arucos in the frame
        robot_pose = robot.get_pose()
        aruco_state = build_aruco_state(frame, depth, camera_intrinsics, robot_pose)    

        if goal_state is not None and aruco_state is not None:
            draw_match_line(frame, aruco_state, goal_state)

        
        if aruco_state is not None:
            for i, goal in enumerate(goal_states.states):
                
                # if state's camera pose is closer than matched state's                
                camera_move_new = match_aruco_state(goal, aruco_state)
                
                if camera_move_new is not None:
                    matched = matched_states[i]
                    if matched is None:
                        camera_move_old = SE3(1e4, 1e4, 1e4)
                    else:
                        camera_move_old = match_aruco_state(goal, matched)
                
                    # if state's overall pose far from matched state's
                    if se3_norm(camera_move_new) < se3_norm(camera_move_old):
                        print("updated matched state for goal", goal.ids)
                        matched_states.states[i] = aruco_state
                        
                    # aruco is moved, update matched state                    
                    elif se3_norm(camera_move_new) < 0.1:
                        marker_pose_new = pose_to_SE3(aruco_state.robot_pose) * hand_eye.gripper_move(camera_move_new)
                        marker_pose_old = pose_to_SE3(matched.robot_pose) * hand_eye.gripper_move(camera_move_old)
                        if se3_norm(marker_pose_new.inv() * marker_pose_old) > 0.02:
                            print("aruco is moved, update matched state for goal", goal.ids)
                            matched_states.states[i] = aruco_state


        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
    
        if key == ord('q'):
            print("Saving matched states number:", len(matched_states.states))
            matched_states.save()
            break

        elif key in key_map:
            code = key_map[key]
            print(f"Action {code}")
            action = lookup_action(code)
            action = hand_eye.gripper_move(action)
            robot.step_in_ee(action=action, wait=False)

        elif key == ord('t'):
            # toggle goal state
            goal_state = goal_states.next_state(goal_state)
            print("Selected goal_state ids:", goal_state.ids)

        elif key == ord('m') and goal_state is not None:
            camera_move = match_aruco_state(goal_state, aruco_state)
            if camera_move is not None:
                gripper_action = hand_eye.gripper_move(camera_move)
                print("Gripper_action")
                gripper_action.printline()
                robot.step_in_ee(action=gripper_action, wait=False)


            if camera_move is None:
                print("No match found in current frame for goal_state:", goal_state.ids)
                # try to find a match in history
                matched = matched_states.states[goal_states.index(goal_state)]
                camera_move = match_aruco_state(matched, goal_state)
                if camera_move is not None:
                    gripper_action = hand_eye.gripper_move(camera_move)
                    action = pose_to_SE3(robot_pose).inv() * pose_to_SE3(matched.robot_pose) * gripper_action
                    robot.step_in_ee(action=action, wait=False)
                


        elif key == ord('g'):
            if aruco_state is None:
                print("No aruco state found")
                continue
            # setup goal
            goal_state = aruco_state
            goal_state.set_type("goal")
            
            print("Saving state at", goal_state.timestamp)
            # dump state to pkl
            goal_states.add_state(goal_state)
            goal_states.save()
            
            # save a image of the frame
            cv2.imwrite(os.path.join(save_data_dir, f"{goal_state.timestamp}.jpg"), frame)


