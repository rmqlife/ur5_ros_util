import rospy
from mySensor.myFTSensor import MyFTSensor
import numpy as np

if __name__ == "__main__":
    rospy.init_node('my_ft_sensor_test_node')
    FTSensor = MyFTSensor(omni_flag=True)
    rospy.sleep(1)  # Adjust the time as needed

    while not rospy.is_shutdown():
        # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )

        rospy.sleep(0.05)
        
        if np.linalg.norm(FTSensor.force) > 0.5:
            print(f"Force detected: {FTSensor.force}")
            # use the force direction to 