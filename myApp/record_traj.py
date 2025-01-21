from myPlanner import MyBag, init_robot_with_ik
import rospy
if __name__ == '__main__':
    rospy.init_node('test_mybag', anonymous=True)
    
    # Initialize the data saver (array_saver)
    mybag = MyBag()
    
    robot = init_robot_with_ik()

    while not rospy.is_shutdown():
        mybag.record('robot pose', robot.get_pose())
        rospy.sleep(1/10)