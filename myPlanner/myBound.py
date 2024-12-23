from .myBag import *
from .pose_util import pose_to_SE3
from .myRobotWithIK import init_robot

def build_bounding_box(filename = "data/boundingbox_traj.json"):
    mybag = MyBag(filename)
    print("loaded traj len:", len(mybag.data['robot pose']))
    # compute the bounding box of the trajectory
    traj = np.array(mybag.data['robot pose'])
    print("traj shape:", traj.shape)
    upper = np.max(traj, axis=0)
    lower = np.min(traj, axis=0)
    print("traj min:", upper)
    print("traj max:", lower)

    # save it to the a bounding box key
    mybag.data = dict()
    mybag.data['upper'] = upper
    mybag.data['lower'] = lower
    
    mybag.export_data("config/boundingbox.json")



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
    rospy.init_node('bounding_box_hold', anonymous=False)
    robot = init_robot()
    rospy.sleep(1)  # Adjust the time as needed


    mybag = MyBag("config/boundingbox.json")
    upper = mybag.data['upper']
    lower = mybag.data['lower']
    print("upper bound", upper)
    print("lower bound", lower)


    bound = MyBound(lower=lower, upper=upper, box_dim=3)

    while not rospy.is_shutdown():
        rospy.sleep(0.005)

        # compare the robot pose to the upper and lower bound 
        pose = robot.get_pose()
        
        # Check if the pose is within the specified bounding box
        if not bound.in_the_box(pose):
            # move to the bounds
            corrected_pose = bound.correct_pose_within_bounds(pose, tolerance=0.01)
            print('corrected pose', corrected_pose)
            print('current pose', robot.get_pose())
            robot.hold(0.5)
            robot.goto_pose(pose=pose_to_SE3(corrected_pose), wait=False, coef=1)
            rospy.sleep(0.1)