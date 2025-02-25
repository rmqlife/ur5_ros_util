import rospy
from myEnv import MyEnv, MyStates
from myPlanner.pose_util import se3_to_str

if __name__ == "__main__":
    rospy.init_node('test_env_sub')
    env = MyEnv(is_publisher=False)

    states = MyStates()
    while True:
        # get state
        try:
            state, frame_draw = env.get_state()
            if state is not None and len(state.marker_poses.keys()) > 0:
                if states.push_back(state) > -1:
                    print(f"new state: {len(states.states)}")
                    for id in state.marker_poses.keys():
                        print(f"id: {id}, pose: {se3_to_str(state.marker_poses[id])}")
        except KeyboardInterrupt:
            print("KeyboardInterrupt caught, exiting...")
            break


