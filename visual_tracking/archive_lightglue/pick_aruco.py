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
    history_states = MyArucoStates(os.path.join(save_data_dir, "history_states.pkl"))

    goal_state = goal_states.next_state(None)

    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()

        # detect arucos in the frame
        robot_pose = robot.get_pose()
        aruco_state = build_aruco_state(frame, depth, camera_intrinsics, robot_pose)    

        if goal_state is not None and aruco_state is not None:
            draw_match_line(frame, aruco_state, goal_state)

        if aruco_state is not None:
            history_index = history_states.push_back(aruco_state)
            if history_index>-1:
                print("added new history state, index:", history_index)
        
        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
    
        if key == ord('q'):
            print("Saving history states number:", len(history_states.states))
            history_states.save()
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
                min_norm = 1000
                min_action = None
                for h_state in reversed(history_states.states):
                    camera_move = match_aruco_state(goal_state, h_state)
                    if camera_move is not None:
                        gripper_action = hand_eye.gripper_move(camera_move)
                        from myArucoStates import action_norm
                        camera_move_norm = action_norm(camera_move)
                        action = pose_to_SE3(robot_pose).inv() * pose_to_SE3(h_state.robot_pose) * gripper_action
                        if  camera_move_norm < min_norm:
                            # compare the min_action distance with the goal_state distance
                            if min_action is not None:
                                action_diff = min_action.inv() * action
                                action_diff = np.linalg.norm(action_diff.t)
                                print("action difference", action_diff)
                                if action_diff > 0.02:
                                    print("aruco is moved, use the latest action")
                                    break
                            min_norm = camera_move_norm
                            min_action = action
                        
                print("min action")
                min_action.printline()
                robot.step_in_ee(action=min_action, wait=False)
                

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


