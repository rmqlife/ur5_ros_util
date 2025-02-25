import cv2
from myEnv import MyEnv, get_timestamp, MyStates
import rospy

def stop_recording(states):
    filename = f"states_{len(states.states)}_{get_timestamp()}.pkl"
    states.save(filename)
    states.empty()
    print(f"saved states to {filename}")


if __name__=="__main__":
    rospy.init_node('recording_poses')
    env = MyEnv(is_publisher=False)
    states = MyStates()

    recording = False
    while True:
        state, frame_draw = env.get_state()        
        if recording:
            states.push_back(state)
            # draw a red circle on the frame_draw top left corner
            cv2.circle(frame_draw, (20, 20), 10, (0, 0, 255), -1)
            # put a text on the frame_draw REC
            cv2.putText(frame_draw, "REC", (30, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("Recorder", frame_draw)
        key = cv2.waitKey(1000//60) & 0xFF 
        if key == ord('q'):
            if recording:
                stop_recording(states)
            break

        elif key == ord('p'):
            print(state)

        elif key == ord(' '):
            recording = not recording
            if recording:
                print("recording...")
            else:
                stop_recording(states)
