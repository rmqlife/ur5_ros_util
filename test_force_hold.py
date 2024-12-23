import rospy
from myPlanner import init_robot_with_ik
from mySensor import MyFTSensor
import numpy as np

if __name__ == "__main__":
    rospy.init_node('test_stop', anonymous=False)
    robot = init_robot_with_ik
    FTSensor = MyFTSensor(omni_flag=False)
    rospy.sleep(1)  # Adjust the time as needed

    while not rospy.is_shutdown():
        # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )
        rospy.sleep(0.05)
        if np.linalg.norm(FTSensor.force) > 0.5:
            print(f"Force detected: {FTSensor.force}")
            robot.hold(0.5)