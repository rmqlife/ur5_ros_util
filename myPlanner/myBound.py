from .myBag import *
from .pose_util import pose_to_SE3
from .myRobotWithIK import init_robot

def build_bounding_box(filename = "data/boundingbox_traj.json", save_filename = "../config/boundingbox.json"):
    mybag = MyBag(filename)
    print("loaded traj len:", len(mybag.data['robot_pose']))
    # compute the bounding box of the trajectory
    traj = np.array(mybag.data['robot_pose'])
    print("traj shape:", traj.shape)
    upper = np.max(traj, axis=0)
    lower = np.min(traj, axis=0)
    print("traj min:", upper)
    print("traj max:", lower)

    # save it to the a bounding box key
    newbag = MyBag(save_filename)
    newbag.data = dict()
    newbag.data['upper'] = upper
    newbag.data['lower'] = lower
    


class MyBound:
    def __init__(self, lower, upper, box_dim=3):
        self.lower = lower
        self.upper = upper
        self.box_dim = box_dim

    
    def correct_pose_within_bounds(self, pose, tolerance=0.02):
        """Correct the pose to stay within the given bounds."""
        corrected_pose = pose.copy()
        for i in range(self.box_dim):
            if corrected_pose[i] < self.lower[i]:
                corrected_pose[i] = self.lower[i]+tolerance
            elif corrected_pose[i] > self.upper[i]:
                corrected_pose[i] = self.upper[i]-tolerance
        return corrected_pose

    def in_the_box(self, pose):
        ret = True
        out_dims = []
        for i in range(self.box_dim):
            if self.lower[i] > pose[i] or self.upper[i]< pose[i]:
                ret = False
                out_dims.append(i)
        if not ret:
            print(f"{out_dims} th pose is out of the bounding box")
        return ret 
    

if __name__ == '__main__':
    build_bounding_box(filename="../data/recorded_poses.json", save_filename="../config/boundingbox_gripper.json")