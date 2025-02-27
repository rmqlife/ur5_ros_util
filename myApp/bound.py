import rospy
from myPlanner import MyBag, MyBound, init_robot_with_ik, pose_to_SE3, se3_to_str

def load_boundingbox(bound_filename):
    mybag = MyBag(bound_filename)
    upper = mybag.data['upper']
    lower = mybag.data['lower']
    return MyBound(lower=lower, upper=upper, box_dim=3)

if __name__=="__main__":    
    rospy.init_node('bounding_box_hold', anonymous=False)
    robot = init_robot_with_ik()
    rospy.sleep(1)  # Adjust the time as needed

    bound = load_boundingbox("../config/boundingbox_gripper.json")

    while not rospy.is_shutdown():
        rospy.sleep(0.005)

        # compare the robot pose to the upper and lower bound 
        pose = robot.get_pose()
        
        # Check if the pose is within the specified bounding box
        if not bound.in_the_box(pose):
            # move to the bounds
            corrected_pose = bound.correct_pose_within_bounds(pose, tolerance=0.003)
            print('corrected pose', se3_to_str(corrected_pose))
            print('current pose', se3_to_str(pose))
            robot.hold(0.5)
            robot.goto_pose(pose=corrected_pose, wait=False, coef=1)
            rospy.sleep(0.1)