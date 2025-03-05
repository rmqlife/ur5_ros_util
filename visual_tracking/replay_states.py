from myEnv import MyStates, ArucoState, MyEnv
from myPlanner.pose_util import se3_to_str, Rt_to_SE3, se3_to_euler6d, euler6d_to_se3
import rospy
from follow_aruco import relative_pose
from spatialmath import SE3
import numpy as np
from position_map import same_position

def replay_robot_space(myEnv, states_to_replay):
    for state in states_to_replay:
        # print(state.marker_poses)
        state.robot_pose.printline()
        myEnv.robot.goto_pose(state.robot_pose, wait=True)

def print_marker_ids(states):
    for i, state in enumerate(states):
        print(f"state {i} have ids: {state.marker_poses.keys()}")
        for marker_id in state.marker_poses.keys():
            print(f"marker {marker_id} pose: {se3_to_str(state.marker_poses[marker_id])}")

def action_add(action1, action2):
    #convert to euler6d
    action_new = action1 * action2
    action_new.t = action1.t + action2.t
    return action_new

def replay_marker_poses(myEnv:MyEnv, states, tcp_id=-1, ref_id=-1):
    # current my env has markers:
    state = None
    while state is None:
        state, _ = myEnv.get_state()
        rospy.sleep(1/30)

    ref_pose = None
    for state in states:        
        state_now, _ = myEnv.get_state()
        tcp_pose_new = state_now.marker_poses.get(tcp_id)
        ref_pose_new = state_now.marker_poses.get(ref_id)
        robot_pose_new = state_now.robot_pose
        tcp_pose = state.marker_poses.get(tcp_id)
        ref_pose = state.marker_poses.get(ref_id)
        robot_pose = state.robot_pose

        pose_pattern = "111001"
        if ref_pose is not None and ref_pose_new is not None:
            ref_move = relative_pose(ref_pose, ref_pose_new, pattern=pose_pattern)
        else:
            ref_move = SE3.Rx(0)
        if tcp_pose is not None and tcp_pose_new is not None:
            tcp_move = relative_pose(tcp_pose_new, tcp_pose, pattern=pose_pattern)
        else:
            tcp_move = relative_pose(robot_pose_new, robot_pose, pattern=pose_pattern)
        
        desired_pose = ref_move * tcp_move * robot_pose_new
        pose_error = relative_pose(desired_pose, robot_pose_new, pattern=pose_pattern)
        print(f"pose_error: {se3_to_str(pose_error)}")

        if not same_position(pose_error, SE3.Rx(0), t_threshold=0.02, rad_threshold=0.2):
            myEnv.robot.goto_pose(desired_pose, wait=True)

if __name__ == "__main__":
    import sys
    if len(sys.argv)<2:
        print("Usage: python replay_states.py <path_to_states>")
        exit()
    rospy.init_node('replay_states')
    my_states = MyStates(filename=sys.argv[1])
    myEnv = MyEnv(is_publisher=False)
    

    while True:
        replay_marker_poses(myEnv, my_states.get_states(), tcp_id=0, ref_id=4)
        rospy.sleep(1)
    # replay the states in Robot c-space
    # replay_robot_space(myEnv, my_states.states)