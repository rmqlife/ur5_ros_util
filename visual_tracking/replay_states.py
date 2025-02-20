from myEnv import MyStates, ArucoState, MyEnv
from myPlanner.pose_util import se3_to_str
import rospy
from follow_aruco import relative_pose

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

def replay_marker_poses(myEnv, states_to_replay, tcp_id, ref_id):
    # current my env has markers:
    for _ in range(10):
        state, _ = myEnv.get_state()
        tcp_pose_new = state.marker_poses.get(tcp_id)
        ref_pose_new = state.marker_poses.get(ref_id)
        rospy.sleep(1/30)
    
    if tcp_pose_new is None:
        print(f"failed to find tcp_pose of id {tcp_id}")
        return 
    
    if ref_pose_new is None:
        print(f"failed to find ref_pose of id {ref_id}")
        return 
    
    ref_pose = None
    for state in states_to_replay:
        # check if dst_id is in state
        tcp_pose = state.marker_poses.get(tcp_id)
        if tcp_pose is None:
            continue

        if ref_id in state.marker_poses:
            ref_pose = state.marker_poses[ref_id]
        
        if ref_pose is not None:
            ref_move = relative_pose(ref_pose, ref_pose_new, pattern="111000")
            tcp_move = relative_pose(tcp_pose_new, tcp_pose, pattern="111000")
            action = ref_move * tcp_move
            myEnv.robot.step(action, wait=True)
            state, _ = myEnv.get_state()
            if tcp_id in state.marker_poses:
                tcp_pose_new = state.marker_poses[tcp_id]
            else:
                tcp_pose_new = tcp_pose.copy()

            if ref_id in state.marker_poses:
                ref_pose_new = state.marker_poses[ref_id]


if __name__ == "__main__":
    states_to_replay = MyStates("./data/push_block_new.pkl").states
    print(f"Loaded {len(states_to_replay)} states")
    myEnv = MyEnv("./data/replay_0220.pkl")


    # replay the states in Robot c-space
    # replay_robot_space(myEnv, states_to_replay)

    # matching marker poses to current frame
    replay_marker_poses(myEnv, states_to_replay, tcp_id=0, ref_id=3)
