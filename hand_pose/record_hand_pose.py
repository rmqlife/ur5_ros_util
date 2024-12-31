import cv2
import mediapipe as mp
import numpy as np
import pandas as pd
from util import get_timestamp
from mySensor import load_intrinsics, MyImageSaver, depth_to_colormap
import rospy  
# 相机内参
# fx = 599.5029
# fy = 598.7244
# cx = 323.4041
# cy = 254.9281

if __name__ == "__main__":
    # 获取当前时间
    timestamp = get_timestamp()
    csv_filepath=f'../data/hand_pose_{timestamp}.csv'
    video_filepath = f'../data/hand_pose_{timestamp}.mp4'  # Define video file path

    # Load camera intrinsics from JSON file
    intrinsics = load_intrinsics("../config/camera_intrinsics.json")  # Ensure config_path is defined or passed correctly
    fx = intrinsics['fx']
    fy = intrinsics['fy']
    cx = intrinsics['cx']
    cy = intrinsics['cy']

    rospy.init_node('hand_pose_recording', anonymous=True)
    camera = MyImageSaver(cameraNS='camera')
    frame_rate = 60

    # 初始化MediaPipe手部模块
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False,
                        max_num_hands=2,
                        min_detection_confidence=0.5,
                        min_tracking_confidence=0.5)

    # 初始化绘制工具
    mp_drawing = mp.solutions.drawing_utils
    csv_data=[]
    recording = False


    while True:
        color_image, depth_image = camera.get_frames()

        if color_image is None or depth_image is None:
            continue  # Skip if no frames are available

        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_image)

        if results.multi_hand_landmarks:
            for handedness, hand_landmarks in zip(results.multi_handedness, results.multi_hand_landmarks):

                which_hand = 'Right' if handedness.classification[0].label == 'Right' else 'Left'
                print("label:", handedness.classification[0].label)
                mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                data_pixel,data_point_cloud=[],[]

                for id, landmark in enumerate(hand_landmarks.landmark):
                    x_norm = landmark.x
                    y_norm = landmark.y
                    z_norm = landmark.z
                    
                    x_pix = int(x_norm * rgb_image.shape[1])
                    y_pix = int(y_norm * rgb_image.shape[0])
                    data_pixel.append([x_pix,y_pix])

                    if (id == 0 ):
                        if depth_image[y_pix, x_pix] == 0:
                            continue
                        refer_depth = depth_image[y_pix, x_pix]/1000
                        depth_value = refer_depth 

                    else:
                        # 其余手指根据手腕（id=0）
                        depth_value = refer_depth+z_norm

                    x_3d = (x_pix - cx) * depth_value / fx
                    y_3d = (y_pix - cy) * depth_value / fy
                    z_3d = depth_value 
                    data_point_cloud.append(x_3d)
                    data_point_cloud.append(y_3d)
                    data_point_cloud.append(z_3d)
                    
                    # print(f"{which_hand} Hand - Landmark {id}: ({x_3d}, {y_3d}, {z_3d})")
                
        # Write the current frame to the video file

        # Check for key press events
        key = cv2.waitKey(1000//frame_rate) & 0xFF
        # Start recording video with blank key
        if key == ord(' '):
            if not recording:
                recording = True
                print("Recording started (via keyboard input)")\
                    # Initialize VideoWriter
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec
                video_writer = cv2.VideoWriter(video_filepath, fourcc, frame_rate/2, (640, 480))  # Adjust resolution as needed
                
            else: 
                recording = False
                print("Recording ended (via keyboard input)")
                df = pd.DataFrame(csv_data, columns= [f'{i}_{c}' for i in range(21) for c in ['x_norm', 'y_norm', 'z_norm']])
                df.to_csv(csv_filepath, index=False)
                
                # Release the video writer
                video_writer.release()
                cv2.destroyAllWindows()
                break
        if key == ord('q'):
            break
        if recording:
            #保存正则化的点云数据
            csv_data.append(np.array(data_point_cloud).flatten())
            video_writer.write(color_image)
            # draw a red circle to show recording   
            cv2.circle(color_image, (30, 30), 15, (0, 0, 255), -1)

        depth_colormap = depth_to_colormap(depth_image)
        cv2.imshow("view", cv2.hconcat([color_image, depth_colormap]))

