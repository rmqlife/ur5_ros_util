# given a target force, the action should be use the function compliant_force_model(current force - target_force)
from test_compliant import *
if __name__ == "__main__":

    target_force = [0, 0, 2]
    model = build_ft_model(data_path='data/force_xy_ring_z_tissue.json')

    rospy.init_node('test_force_servo', anonymous=True)
    robot = init_robot()
    FTSensor = MyFTSensor(omni_flag=False)
    while not rospy.is_shutdown():
        # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )
        rospy.sleep(0.01)
        force = FTSensor.force
        force_delta = target_force - force
        if np.linalg.norm(force_delta) > 0.5:
            print(f"Force delta detected: {force_delta}")
            action = model.predict(force_delta)
            print(f"Predicted action: {action}")
            robot.step(action=pose_to_SE3(action), wait=False, coef=1)