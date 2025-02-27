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

# using roboticstoolbox SE3 class
class MyRobotWithIK(MyRobot):
    def __init__(self, myIK: MyIK):
        self.myIK = myIK
        super().__init__()

    def get_pose(self):
        return self.myIK.fk(super().get_joints())
    
    def goto_pose(self, pose:SE3, wait, coef=3):
        joints = super().get_joints()
        joints_star = self.myIK.ik(pose, q=joints)
        # compute the difference between joints and joints_star
        joints_star = angle_transfer(joints, joints_star)
        return super().move_joints_smooth(joints_star, coef=coef, joint_thresh=2, wait=wait)

    def step_in_ee(self, action:SE3, wait:bool):
        """
        relative move in ee frame
        """
        pose_se3 = self.get_pose()
        pose_se3_new =  pose_se3 * action # right multiply
        return self.goto_pose(pose_se3_new, wait)

    def step(self, action:SE3, wait:bool):
        '''
        if wait ==True:
            wait until reach targets
        '''
        pose = self.get_pose()
        # pose_new = action * pose
        R = (action * pose).R
        t = action.t + pose.t
        pose_new = Rt_to_SE3(R, t)
        return self.goto_pose(pose_new, wait)


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