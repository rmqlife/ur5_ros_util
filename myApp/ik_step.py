import numpy as np
from spatialmath import SE3
from myPlanner import init_robot_with_ik
from myPlanner.pose_util import *
from mySensor import MyImageSaver

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

def lookup_action(code, t_move=0.005, r_move=1):
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

if __name__ == "__main__":
    rospy.init_node('ik_step', anonymous=True)
    image_saver = MyImageSaver(cameraNS='camera')
    framedelay = 1000//20

    robot = init_robot_with_ik()
    
    
    while not rospy.is_shutdown():
        frame, depth = image_saver.get_frames()
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