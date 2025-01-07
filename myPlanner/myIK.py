import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from .pose_util import *
import math
from spatialmath import SE3

unit_vector = [1,0,0]
def rectangle_points(center, x, y):
    points = []
    for i, j in [(-x,-y), (-x, y), (x,y), (x, -y),(-x,-y)]:
        point = center.copy()
        point[0]+=i
        point[1]+=j
        points.append(point)
    
    # connect start to end    
    return points

def vec_to_quat(v1, v2):
    import tf.transformations as transformations
    # Ensure input vectors are unit vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    # Compute the quaternion representing the rotation from v1 to v2
    q = transformations.quaternion_about_axis(np.arccos(np.dot(v1, v2)), np.cross(v1, v2))
    # q = swap_quat(q)
    return q


def circle_points(center, radius=0.1, num_points=20):
    points = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        point = np.array([center[0] + delta_x, center[1] + delta_y, center[2]])
        points.append(point)

    return points


def circle_pose(center, toward,  radius, num_points):
    points = circle_points(center, radius=radius, num_points=num_points)
    for i in range(len(points)):
        quat = vec_to_quat(unit_vector,  toward-points[i])
        points[i] =  list(points[i][:3]) + list(quat)

    return np.array(points)


class MyIK:
    def __init__(self):
        self.robot = rtb.models.UR5()
    
    def fk_se3(self, q):
        return self.robot.fkine(q)

    def ik_se3(self, se3, q):
        return self.robot.ikine_LM(se3, q0=q).q

    def fk(self, q):
        return SE3_to_pose(self.fk_se3(q))

    def ik(self, pose, q):
        return self.ik_se3(pose_to_SE3(pose), q)

    def ik_point(self, point, q):
        current_se3 = self.robot.fkine(q)
        current_se3.t = [0,0,0]
        desired_pose = SE3(point) @ SE3(current_se3)
        return  self.robot.ikine_LM(desired_pose, q0=q).q
        
    def plan_trajectory(self, poses, q, vel_threshold=0.02):
        poses = np.array(poses)
        traj = None
        if poses.shape[1]==3:
            # it is a points
            init_pose = self.fk(q)
            poses = append_vector(poses, init_pose[3:])

        for i in range(len(poses)-1):
            start_pose = poses[i]
            end_pose = poses[i+1]
            # linear velocity 
            vel = np.linalg.norm(np.array(start_pose[:3])-np.array(end_pose[:3]))
            if vel<vel_threshold:
                num_steps=1
            else:
                print('vel threshold', vel_threshold)
                num_steps=int(math.floor(vel/vel_threshold))
            
            start_joint_angles = self.ik(start_pose, q)
        
            end_joint_angles = self.ik(end_pose, q)
            traj_segment = rtb.jtraj(start_joint_angles, end_joint_angles, num_steps)
            if traj is None:
                traj = traj_segment.q
            else:
                traj = np.concatenate((traj, traj_segment.q), axis=0)
        return traj

    def show_traj(self, traj, loop=True):
        self.robot.plot(traj, backend='pyplot', loop=loop)



if __name__ == "__main__":
    joints = [-1.57, -1.57, 1.57, -1.57, -1.57, 0]

    myIK = MyIK()

    init_pose = myIK.fk(joints)
    ax = visualize_poses(init_pose, label='init pose', autoscale=False, ax=None)
    target_pose = init_pose.copy()
    target_pose[2] -= 0.3
    poses = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=50)
    # points = circle_points(init_pose[:3], radius=0.2, num_points=50)
    # points = rectangle_points(init_pose[:3], x=0.1, y=0.1)
    visualize_poses(poses, label="points to plan", color='y', autoscale=False, ax=ax)

    traj = myIK.plan_trajectory(poses, joints)
    myIK.show_traj(traj,loop=True)



