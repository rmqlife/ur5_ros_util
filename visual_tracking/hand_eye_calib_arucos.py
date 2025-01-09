import numpy as np
from myPlanner.pose_util import *

import pickle
def save_object(obj, file_path):
    with open(file_path, 'wb') as file:
        pickle.dump(obj, file)

def load_object(file_path):
    with open(file_path, 'rb') as file:
        obj = pickle.load(file)
    return obj

def pose_to_T(pose):
    R, t = pose_to_Rt(pose)
    return Rt_to_T(R, t)

def Rt_to_T(R, t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.flatten()
    return T

def T_to_pose(T):
    t = T[:3, 3]
    R = T[:3, :3]
    q = R_to_quat(R)
    return np.concatenate((t, q))

class MyHandEye:
    def __init__(self):
        pass

    def save(self, filename):
        data = {
            'T_c2g': self.T_c2g,
            'T_t2c': self.T_t2c,
            'T_g2b': self.T_g2b,
        }
        np.savez(filename, **data)

    def load(self, filename):
        data = np.load(filename)
        self.T_c2g = data['T_c2g']
        self.T_t2c = data['T_t2c']
        self.T_g2b = data['T_g2b']

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
        # # translate gripper pose to camera pose



    def compute_t2b(self):
        T_t2b= []
        for A,B in zip(self.T_g2b,self.T_t2c):
            T_t2b.append(A @ self.T_c2g @ B)
        return T_t2b
    

def compute_model(marker_poses, robot_poses, method='eye_in_hand'):
    if method == 'eye_in_hand':
        myHandEye = MyHandEye()
        myHandEye.eye_in_hand(poses_m2c=marker_poses, poses_g2b=robot_poses)
        myHandEye.save('./hand_eye.npz')
        return myHandEye
    
    elif method == 'eye_to_hand':
        marker_poses2 = np.load(f'{folder}/marker_poses2.npy')
        T_t2c2 = pose_to_T(marker_poses2[0])
        return myHandEye, T_t2c2

def validate_model(model_path):
    # test 
    myHandEye = MyHandEye()
    myHandEye.load(model_path)
    T_c2g = myHandEye.T_c2g
    T_t2b = myHandEye.compute_t2b()
    for i in range(len(T_t2b)):
        T_slam = SE3(T_t2b[i])
        T_slam.printline()


def Ts_to_poses(Ts):
    poses = []
    for T in Ts:
        pose = T_to_pose(T).tolist()
        poses.append(pose)
    return poses

def poses_to_Ts(poses):
    Ts = []
    for pose in poses:
        T = pose_to_T(pose)
        Ts.append(T)
    Ts = np.array(Ts)
    return Ts

def load_shez_data():
    folder = 'data/images-20241126-155752'
    marker_poses = np.load(f'{folder}/marker_poses.npy')
    robot_poses = np.load(f'{folder}/robot_poses.npy')
    return marker_poses, robot_poses


def load_mybag_poses():
    from myPlanner import MyBag
    bag = MyBag(filename='data/images-20250109-172550/record_aruco.json')
    robot_poses = bag.data["robot_pose"]
    marker_poses = bag.data["marker_pose"]

    marker_poses = poses_to_Ts(marker_poses)
    robot_poses = poses_to_Ts(robot_poses)

    print(marker_poses.shape, robot_poses.shape)
    return marker_poses, robot_poses

if __name__ == "__main__":

    marker_poses, robot_poses = load_mybag_poses()

    myHandEye= compute_model(marker_poses=marker_poses, robot_poses=robot_poses, method='eye_in_hand')
    T_t2b = myHandEye.compute_t2b()
    # print(SE3(myHandEye.T_c2g))
    for i in range(len(T_t2b)):
        T_b2t = SE3(T_t2b[i]).inv() # select one as T_t2b
        T_b2t.printline()
    # T_b2t = SE3(T_t2b[0]).inv() # select one as T_t2b
    # print(T_b2t)
    print("\\\\\\\\")
    # save_object(T_b2t, f'{data_dir}/base_transform.pkl')
    # validate_model(model_path=f'{data_dir}/hand_eye.npz')



