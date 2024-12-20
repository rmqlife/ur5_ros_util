import myBag
import numpy as np
from sklearn.linear_model import Lasso
from sklearn.metrics.pairwise import euclidean_distances
from pose_util import *
from optimize_util import myLinearProg
from myRobotWithIK import init_robot
from mySensor.myFTSensor import MyFTSensor
import rospy

FORCE_THRESHOLD = 0.3


def normalize_to_fixed_length(force, length=1.0):
    """
    Normalize each force vector to have a fixed length (default is 1.0).
    """
    norm = np.linalg.norm(force)  # Compute the L2 norm of each force vector
    # Avoid division by zero
    return force / norm * length  # Scale the force to the desired length


def clean_bag(mybag, force_threshold):
    force = np.array(mybag.data['force'])
    action = np.array(mybag.data['action'])
    force = force.reshape(force.shape[0], -1)
    action = action.reshape(action.shape[0], -1)
    print("Size of force data:", force.shape)
    print("Size of action data:", action.shape)
    # Reshape the data for the model
    
    force_delta = force.copy()
    # use delta force to represent the force
    for i in range(1, len(force)):
        force_delta[i] = force[i-1] - force[i]

    # clean out the force that is too small
    idx = np.linalg.norm(force_delta, axis=1) > force_threshold
    force_delta = force_delta[idx]
    action = action[idx]
    return force_delta, action

def build_model_lasso(forces,actions):

    # Create and train the model with Lasso
    model = Lasso(alpha=0.001, max_iter=100000)
    model.fit(forces, actions)
    return model


def find_closest_action(new_force, forces, actions):
    """
    Find the closest action for the given new force by calculating the Euclidean distance to all forces.
    """
    new_force = normalize_to_fixed_length(new_force)
    new_force = np.array(new_force).reshape(1, -1)
    # Compute the Euclidean distance between the new force and each force in the dataset
    distances = euclidean_distances(new_force, forces)
    
    print('distance', distances)
    # Find the index of the smallest distance (closest force)
    closest_idx = np.argmin(distances)
    
    # Return the corresponding action for the closest force
    closest_action = actions[closest_idx]
    return closest_action

class FTModel():
    def __init__(self, forces, actions):
        action_force_dict = {}
        for action, force in zip(actions, forces):
            action_key = tuple(action)
            if action_key in action_force_dict:
                action_force_dict[action_key].append(force)
            else:
                action_force_dict[action_key] = [force]

        action_new = list()
        force_new = list()
        for action, val in action_force_dict.items():
            force = np.mean(val, axis=0)
            force = normalize_to_fixed_length(force)
            print(f"action: {action}, Mean: {force}")
            action_new.append(np.array(action))
            force_new.append(force)
        self.actions = np.array(action_new)
        self.forces = np.array(force_new) 

    def predict(self, new_force):
        coef = myLinearProg(self.forces.T, new_force)
        print("FTModel coef", coef)
        new_action = np.dot(coef, self.actions)
        new_action *= 0.3
        return new_action

if __name__ == "__main__":
    mybag = myBag.MyBag('data/force_xy_ring_z_tissue.json')
    forces, actions = clean_bag(mybag, force_threshold=FORCE_THRESHOLD)

    model = FTModel(forces, actions)

    # Example prediction
    # Assuming we want to predict the action for a new force measurement
    new_force = np.array([0.010287844575941563, -0.005950129125267267, 20])
    new_action = model.predict(new_force)
    print("new action", new_action)


    rospy.init_node('test_compliant_ctr', anonymous=True)
    robot = init_robot()
    FTSensor = MyFTSensor(omni_flag=False)
    while not rospy.is_shutdown():
        # print(f"force{FTSensor.force}, torque{FTSensor.torque}", )
        rospy.sleep(0.01)
        new_force = FTSensor.force
        if np.linalg.norm(new_force) > 1:
            print(f"Force detected: {new_force}")
            new_action = find_closest_action(new_force, forces, actions)

            new_action = model.predict(new_force)
            print(f"Predicted action: {new_action}")
            robot.step(action=pose_to_SE3(new_action), wait=False, coef=1)
