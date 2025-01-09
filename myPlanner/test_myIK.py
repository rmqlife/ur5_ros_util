from myPlanner import MyIK
from pose_util import visualize_poses
from myPlanner.myIK import circle_pose, circle_points, rectangle_points
from matplotlib import pyplot as plt
from spatialmath import SE3
if __name__ == "__main__":
    joints = [-1.57, -1.57, 1.57, -1.57, -1.57, 0]

    myIK = MyIK()
    # myIK.set_base(SE3(0,0,0.3))

    init_pose = myIK.fk(joints)
    ax = visualize_poses(init_pose, label='init pose', autoscale=False, ax=None)
    target_pose = init_pose.copy()
    target_pose[2] -= 0.3
    poses = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=50)
    # points = circle_points(init_pose[:3], radius=0.2, num_points=50)
    # points = rectangle_points(init_pose[:3], x=0.1, y=0.1)
    visualize_poses(poses, label="points to plan", color='y', autoscale=False, ax=ax)
    print(poses.shape)
    # plt.show()
    traj = myIK.plan_trajectory(poses, joints)
    myIK.show_traj(traj,loop=True)