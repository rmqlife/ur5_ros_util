from myEnv import MyStates, ArucoState, MyEnv
from myPlanner.pose_util import se3_to_str
import rospy
from follow_aruco import relative_pose
from spatialmath import SE3

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

def replay_marker_poses(myEnv, states, tcp_id, ref_id):
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
        tcp_pose = state.marker_poses.get(tcp_id)
        ref_pose = state.marker_poses.get(ref_id)

        if ref_pose is not None and ref_pose_new is not None:
            ref_move = relative_pose(ref_pose, ref_pose_new, pattern="111000")
        else:
            ref_move = SE3.Rx(0)
        if tcp_pose is not None and tcp_pose_new is not None:
            tcp_move = relative_pose(tcp_pose_new, tcp_pose, pattern="111000")
        else:
            tcp_move = SE3.Rx(0)
        action = ref_move * tcp_move
        myEnv.robot.step(action, wait=True)

if __name__ == "__main__":
    import sys
    if len(sys.argv)<2:
        print("Usage: python replay_states.py <path_to_states>")
        exit()
    rospy.init_node('replay_states')
    my_states = MyStates(filename=sys.argv[1])
    myEnv = MyEnv(is_publisher=False)
    
    replay_marker_poses(myEnv, my_states.get_states(), tcp_id=0, ref_id=4)
    
    # replay the states in Robot c-space
    # replay_robot_space(myEnv, my_states.states)