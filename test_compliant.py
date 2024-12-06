import myBag
import numpy as np
from sklearn.linear_model import Lasso
from pose_util import *

def build_model(mybag):
    force_data = np.array(mybag.data['force'])
    action_data = np.array(mybag.data['action'])

    print("Size of force data:", force_data.shape)
    print("Size of action data:", action_data.shape)
    # Reshape the data for the model
    force_data = force_data.reshape(force_data.shape[0], -1)
    action_data = action_data.reshape(action_data.shape[0], -1)

    
    for i in range(1, len(force_data)):
        force_data[i] = force_data[i-1] - force_data[i]

    # Create and train the model with Lasso
    model = Lasso(alpha=0.001, max_iter=100000)
    model.fit(force_data, action_data)
    return model

def test_model_online(model):
    pass

if __name__ == "__main__":
    mybag = myBag.MyBag('data/force_action_z_wait.json')
    # where each sublist in 'force' corresponds to a sublist in 'action'
    model = build_model(mybag)

    # Example prediction
    # Assuming we want to predict the action for a new force measurement
    new_force = np.array([[0.010287844575941563, -0.005950129125267267, -20]])
    new_force = new_force.reshape(1, -1)  # Reshape for the model
    predicted_action = model.predict(new_force)[0]
    print("Predicted action for the given force:", predicted_action)


    if True:
        from myRobotWithIK import init_robot
        from mySensor.myFTSensor import MyFTSensor
        import rospy
        rospy.init_node('test_stop', anonymous=False)
        robot = init_robot()
        robot.step(pose_to_SE3(predicted_action), wait=False)
        FTSensor = MyFTSensor(omni_flag=False)
        while not rospy.is_shutdown():
            # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )
            rospy.sleep(0.05)
            force = FTSensor.force
            if np.linalg.norm(FTSensor.force) > 0.5:
                print(f"Force detected: {force}")
                action = model.predict(force.reshape(1, -1))[0]
                print(f"Predicted action: {action}")
                robot.step(action=pose_to_SE3(action), wait=False)