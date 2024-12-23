from myBag import *
if __name__ == '__main__':
    rospy.init_node('test_mybag', anonymous=True)
    
    # Initialize the data saver (array_saver)
    mybag = MyBag()
    
    from myRobotWithIK import init_robot
    robot = init_robot()

    while not rospy.is_shutdown():
        mybag.record('robot pose', robot.get_pose())
        rospy.sleep(1/10)