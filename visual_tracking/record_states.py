import cv2
from myEnv import MyEnv, get_timestamp

if __name__=="__main__":
    print(get_timestamp())
    import sys
    if len(sys.argv)>1:
        env = MyEnv(sys.argv[1])
    else:
        env = MyEnv(get_timestamp()+".pkl")

    while True:
        state, frame_draw = env.get_state()
        env.states.push_back(state)

        cv2.imshow("camera", frame_draw)
        key = cv2.waitKey(1000//60) & 0xFF 
            
        if key == ord('q'):
            env.states.save()
            print(f"\n\n saving {len(env.states.states)} states")
            break

        elif key == ord('p'):
            print(env.position_map)