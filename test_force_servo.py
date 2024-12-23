# given a target force, the action should be use the function compliant_force_model(current force - target_force)
from tests.test_compliant import *
if __name__ == "__main__":

    target_force = [0, 0, -0.5]
    model = build_ft_model(data_path='data/force_xy_ring_z_tissue.json')

    rospy.init_node('force_servoing123', anonymous=False)
    robot = init_robot_with_ik()
    FTSensor = MyFTSensor(omni_flag=False)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )
        rospy.sleep(0.005)
        force = FTSensor.force
        force_delta = force - target_force
        if np.linalg.norm(force_delta) > 0.2:
            print(f"Force delta detected: {force_delta}")
            action = model.predict(force_delta)
            action *= 0.05
            print(f"Predicted action: {action}")
            robot.step(action=pose_to_SE3(action), wait=False, coef=3)
