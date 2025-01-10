import numpy as np
from .pose_util import *
from .myIK import MyIK
from .myRobot import MyRobot
import rospy

def angle_transfer(joints1, joints2):
    for i in range(len(joints2)):
        while joints2[i] - joints1[i] >= np.pi:
            joints2[i] -= 2 * np.pi
        while joints2[i] - joints1[i] < -np.pi:
            joints2[i] += 2 * np.pi
    return joints2


class MyRobotWithIK(MyRobot):
    def __init__(self, myIK):
        self.myIK = myIK
        super().__init__()


    def get_pose(self):
        return self.myIK.fk(super().get_joints())
    
    def get_pose_se3(self):
        return pose_to_SE3(self.get_pose())
    
    # in SE3 format
    def goto_pose(self, pose, wait, coef=3):
        joints = super().get_joints()
        joints_star = self.myIK.ik_se3(pose, q=joints)
        # compute the difference between joints and joints_star
        joints_star = angle_transfer(joints, joints_star)
        return super().move_joints_smooth(joints_star, coef=coef, wait=wait)

    def step_in_ee(self, action, wait):
        """
        relative move in ee frame
        """
        pose_se3 = pose_to_SE3(self.get_pose())
        pose_se3_new =  pose_se3 * action # right multiply
        
        return self.goto_pose(pose_se3_new, wait)


    def step(self, action, wait, coef=3):
        '''
        if wait ==True:
            wait until reach targets
        '''
        pose_se3 = pose_to_SE3(self.get_pose())
        # print('action print'), action.printline()
        pose_se3_new = action * pose_se3  #action is based on base frame
        pose_se3_new.t = pose_se3.t + action.t
        
        if self.goto_pose(pose_se3_new, wait, coef=coef)==True:
            return pose_se3_new
        else:
            return pose_se3
    
    def step_duration(self, action, duration):
        pose_se3 = pose_to_SE3(self.get_pose())
        # print('action print'), action.printline()
        pose_se3_new = action * pose_se3  #action is based on base frame
        pose_se3_new.t = pose_se3.t + action.t
        
        joints = super().get_joints()
        joints_star = self.myIK.ik_se3(pose_se3_new, q=joints)
        # compute the difference between joints and joints_star
        joints_star = angle_transfer(joints, joints_star)
        return super().move_joints(joints_star, duration=duration, wait=False)


    def goto_poses(self, poses, dry_run, coef=3):
        joints = super().get_joints()
        traj = self.myIK.plan_trajectory(poses, joints)
        if dry_run:
            self.myIK.show_traj(traj, loop=dry_run)
        else:
            for joints_star in traj:
                joints = super().get_joints()
                super().move_joints_smooth(joints_star, coef=coef, wait=False)


def init_robot():
    return MyRobotWithIK(MyIK())



if __name__ == "__main__":
    dry_run = True
    rospy.init_node('test_with_IK', anonymous=True)
    
    from myIK import *
    robot = init_robot()

    init_pose = robot.get_pose()
    ax = visualize_poses(init_pose, label='init pose', autoscale=False, ax=None)
    target_pose = init_pose.copy()
    target_pose[2] -= 0.3
    
    # points = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=100)
    # points[:, 5] = init_pose[5]

    # points = rectangle_points(init_pose[:3], x=0.1, y=0.1)
    points = circle_points(init_pose[:3], radius=0.1, num_points=50)
    visualize_poses(points, label="points to plan", color='y', autoscale=False, ax=ax)
    for _ in range(5):
        robot.goto_poses(points, dry_run=False, coef=3)