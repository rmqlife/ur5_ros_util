import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from util import *


def get_gripper_scale(distance):
    #each set is 3 numbers get the distance between the two points(4 and 8)
    gripper_scale=1000*(distance)/0.08
    gripper_scale = max(0, min(gripper_scale, 1000))  # Clamp gripper_scale between 0 and 1000
    gripper_scale = (gripper_scale // 200) * 200     # Round down to the nearest multiple of 200
    return gripper_scale


class HandTraj:
    def __init__(self, path) -> None:
        df = pd.read_csv(path)
        self.data = df.iloc[:]


    def get_keypoints(self):
        '''
        0,1,5,9,13,17
        '''
        landmarks_list = []
        for index, row in self.data.iterrows():
            landmarks = []
            for i in [0,1,5,13,17]:
                points = SE3.Tx(row[f"{i}_x_norm"]) @ SE3.Ty(row[f"{i}_y_norm"]) @ SE3.Tz(row[f"{i}_z_norm"])
                transformations = SE3.Rz(-90,unit='deg') 
                new_points =  transformations * points 
                landmarks.append([new_points.t[0], new_points.t[1], new_points.t[2]])
            landmarks_list.append(landmarks.copy())

        return np.array(landmarks_list)
    
    def get_simulated_gripper_size(self):
        scale = []
        for index, row in self.data.iterrows():
            thumb_tip = np.array([row["4_x_norm"], row["4_y_norm"], row["4_z_norm"]])
            index_tip = np.array([row["8_x_norm"], row["8_y_norm"], row["8_z_norm"]])
            distance = np.linalg.norm(thumb_tip - index_tip)
            scale.append(get_gripper_scale(distance))
        return scale
    
    def draw_carton(self, keypoints,delay=0.5):
        data = keypoints
        print("shape", data.shape)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(-1, 1)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        for i in range(len(data)):
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")
            ax.scatter(data[i][:, 0], data[i][:, 1], data[i][:, 2])  # 绘制当前帧的数据
            plt.draw()  # 更新图形
            plt.pause(delay)  # 暂停以展示当前帧
            if i < len(data) - 1:
                # ax.cla()  # 清除当前图像内容，为下一帧准备
                ax.set_xlim(-1, 1)
                ax.set_ylim(-1, 1)
                ax.set_zlim(-1, 1)
        
        plt.show()  # 展示最后一帧
        plt.close()  # 关闭图形
    

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        print("Please provide a CSV file as an argument.")
        sys.exit(1)

    hand_traj = HandTraj(csv_file)
    keypoints = hand_traj.get_keypoints()
   
    gripper_scale = hand_traj.get_simulated_gripper_size()
    print(gripper_scale)
    SE3_poses = []
    for i in range(len(keypoints)-1):
        T = find_transformation(keypoints[i], keypoints[i+1])
        SE3_poses.append(T)
    smooth_SE3 = smooth_trajectory(SE3_poses)

    dry_run = True
    if dry_run:
        draw_movingframe(smooth_SE3, keypoints)
    else:
        import rospy
        rospy.init_node('dino_bot')
        # object move
        from myPlanner import init_robot_with_ik
        robot = init_robot_with_ik()
        for action in smooth_SE3: # in range(len(smooth_SE3)):
            print(action)
            robot.step(action, wait=False)
            rospy.sleep(0.05)
