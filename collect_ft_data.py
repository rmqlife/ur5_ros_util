from ik_step import *

if __name__ == "__main__":
    rospy.init_node('collect_ft_data', anonymous=True)
    image_saver = MyImageSaver(cameraNS='camera')
    framedelay = 1000//20

    robot = init_robot_with_ik()
    mybag = MyBag()
    from mySensor.myFTSensor import MyFTSensor
    ft_sensor = MyFTSensor()    
    
    
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

            mybag.record("action", SE3_to_pose(action))
            mybag.record("pose", SE3_to_pose(se3_pose))
            mybag.record("force", ft_sensor.force)
            mybag.record("torque", ft_sensor.torque)