from myPlanner import MyBag, init_robot_with_ik
import rospy
if __name__ == '__main__':
    rospy.init_node('record_poses', anonymous=True)
    
    # Initialize the data saver (array_saver)
    mybag = MyBag("../data/recorded_poses.json")
    
    robot = init_robot_with_ik()

    while not rospy.is_shutdown():
        mybag.record('robot_pose', robot.get_pose())
        rospy.sleep(1/10)
        
