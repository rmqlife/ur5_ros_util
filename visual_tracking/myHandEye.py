import numpy as np
from myPlanner.pose_util import *
import json

class MyHandEye:
    def __init__(self, filename=None):
        if filename is not None:
            self.load(filename)

    def save(self, filename):
        data = {
            'T_c2g': self.T_c2g.tolist()
        }
        with open(filename, 'w') as file:
            json.dump(data, file)
    
    def load(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
            self.T_c2g = SE3(np.array(data['T_c2g']))
            

    def eye_in_hand(self, poses_m2c, poses_g2b):
        self.T_t2c = []
        for p in poses_m2c:
            self.T_t2c.append(np.array(p))

        self.T_g2b = []
        for p in poses_g2b:
            self.T_g2b.append(np.array(p))
     
        R_gripper2base = [a[:3, :3] for a in self.T_g2b]
        t_gripper2base = [a[:3, 3] for a in self.T_g2b]
        R_target2cam = [b[:3, :3] for b in self.T_t2c]
        t_target2cam = [b[:3, 3] for b in self.T_t2c]

        import cv2
        R_c2g, t_c2g = cv2.calibrateHandEye(R_gripper2base=R_gripper2base, t_gripper2base=t_gripper2base, R_target2cam=R_target2cam, t_target2cam=t_target2cam)
        self.T_c2g = Rt_to_T(R_c2g, t_c2g)
        print("camera to gripper: \n",SE3(self.T_c2g))

        return self.T_c2g

    def compute_t2b(self):
        T_t2b= []
        for A,B in zip(self.T_g2b, self.T_t2c):
            T_t2b.append(A @ self.T_c2g @ B)
        return T_t2b
    
    def gripper_move(self, camera_move):
        return self.T_c2g * camera_move * self.T_c2g.inv()



def load_mybag_poses(filename):
    from myPlanner import MyBag
    bag = MyBag(filename=filename)
    robot_poses = bag.data["robot_pose"]
    marker_poses = bag.data["marker_pose"]

    marker_poses = poses_to_Ts(marker_poses)
    robot_poses = poses_to_Ts(robot_poses)

    print(marker_poses.shape, robot_poses.shape)
    return marker_poses, robot_poses

if __name__ == "__main__":

    marker_poses, robot_poses = load_mybag_poses(filename='data/images-20250109-172550/record_aruco.json')

    myHandEye = MyHandEye()
    myHandEye.eye_in_hand(poses_m2c=marker_poses, poses_g2b=robot_poses)
    myHandEye.save('./hand_eye.json')

    T_t2b = myHandEye.compute_t2b()

    for i in range(len(T_t2b)):
        T_b2t = SE3(T_t2b[i]).inv() # select one as T_t2b
        T_b2t.printline()
    T_b2t = SE3(T_t2b[0]).inv() # select one as T_t2b

    print("base_transform:", T_b2t)


