import numpy as np
from pose_util import *
from spatialmath import SE3
from myIK import MyIK
from myRobot import MyRobot
from myImageSaver import MyImageSaver
import rospy
import cv2

key_map = {
    ord('!'): -1,  # Shift+1
    ord('@'): -2,  # Shift+2
    ord('#'): -3,  # Shift+3
    ord('$'): -4,  # Shift+4
    ord('%'): -5,  # Shift+5
    ord('^'): -6,  # Shift+6
    ord('1'): 1,   # 1
    ord('2'): 2,   # 2
    ord('3'): 3,   # 3
    ord('4'): 4,   # 4
    ord('5'): 5,   # 5
    ord('6'): 6    # 6
}

def lookup_action(code, t_move=0.02, r_move=5):
    if abs(code)<=3:
        movement = t_move * np.sign(code)
    else:
        movement = r_move * np.sign(code)
    if abs(code)==1:
        return SE3.Tx(movement)
    elif abs(code)==2:
        return SE3.Ty(movement)
    elif abs(code)==3:
        return SE3.Tz(movement)
    elif abs(code)==4:
        return SE3.Rx(movement, unit='deg')
    elif abs(code)==5:
        return SE3.Ry(movement, unit='deg')
    elif abs(code)==6:
        return SE3.Rz(movement, unit='deg')
    return None


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
        
        print("new pose SE3:" ,pose_se3_new)
        return self.goto_pose(pose_se3_new, wait)


    def step(self, action, wait):
        '''
        if wait ==True:
            wait until reach targets
        '''
        pose_se3 = pose_to_SE3(self.get_pose())
        # print('action print'), action.printline()
        pose_se3_new = action * pose_se3  #action is based on base frame
        # if np.linalg.norm(action.t)<0.001:
        #     # rotation keep the x, y, z
        pose_se3_new.t = pose_se3.t + action.t
        
        if self.goto_pose(pose_se3_new, wait)==True:
            return pose_se3_new
        else:
            return pose_se3
        

    def goto_poses(self, poses, dry_run, coef=3):
        joints = super().get_joints()
        traj = self.myIK.plan_trajectory(poses, joints)
        self.myIK.show_traj(traj, loop=dry_run)
        if not dry_run:
            for joints_star in traj:
                joints = super().get_joints()
                super().move_joints_smooth(joints_star, coef=coef, wait=False)


if __name__ == "__main__":
    rospy.init_node('ik_step', anonymous=True)
    image_saver = MyImageSaver(cameraNS='camera')
    framedelay = 1000//20

    robot = MyRobotWithIK(myIK=MyIK())    

    while not rospy.is_shutdown():
        frame = image_saver.rgb_image
        cv2.imshow('Camera', frame)
        key = cv2.waitKey(framedelay) & 0xFF 
        if key == ord('q'):
            break
        elif key in key_map:
            code  = key_map[key]
            print(f"action {code}")
            action = lookup_action(code)
            se3_pose = robot.step(action=action, wait=False)
            se3_pose.printline()
            image_saver.record()