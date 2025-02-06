from myPlanner.myBound import build_bounding_box
from bound import load_boundingbox
from myPlanner import init_robot_with_ik, pose_to_SE3
import rospy
if __name__ == '__main__':
    save_filename = "../config/boundingbox_gripper.json"
    build_bounding_box(filename="../data/recorded_poses.json", save_filename=save_filename)
    
    bound = load_boundingbox(save_filename)
    
    # move the robot to the 8 corners of boundingbox
    rospy.init_node('test_boundingbox', anonymous=True)
    robot = init_robot_with_ik()
    poses = []
    for i in range(2):
        for j in range(2):
            for k in range(2):
                corner_pose = [bound.lower[0] if i == 0 else bound.upper[0], 
                               bound.lower[1] if j == 0 else bound.upper[1], 
                               bound.lower[2] if k == 0 else bound.upper[2]]
                corner_pose = corner_pose + robot.get_pose()[3:]
                poses.append(corner_pose)
    # print("poses", poses)
                # print("corner pose", corner_pose)

    for pose in poses:
        print("pose", pose)
        robot.goto_pose(pose_to_SE3(pose), wait=True, coef=3)
        rospy.sleep(1)
