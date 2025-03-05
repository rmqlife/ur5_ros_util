import numpy as np
import math
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from spatialmath import SE3
from spatialmath.base import trnorm

unit_vector = [1,0,0]

def normalize_vector(vector, l):
    # Calculate the length of the vector
    vector_length = np.linalg.norm(vector)
    # Normalize the vector
    normalized_vector = (vector / vector_length) * l
    return normalized_vector

def visualize_poses(poses, label="None", color='b', ax=None, autoscale=False):
    if ax is None:
        ax = plt.axes(projection ='3d')
        # Plot SLAM poses
        ax.set_autoscale_on(autoscale)

    poses = np.array(poses)
    if len(poses.shape)<2:
        poses = np.array([poses])
    print(f'loaded {poses.shape} poses to visualize')
    # plot positions
    p = poses[:, :3]
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=color, marker='o', label=label)

    if poses.shape[1]>3:
        # have orientation
        q = poses[:, 3:]
        # Plot orientation vectors
        for i in range(len(p)):
            # Convert quaternion to rotation matrix
            d = quat_to_R(q[i]) @ unit_vector
            d = normalize_vector(d, l=0.2)
            ax.quiver(p[i, 0], p[i, 1], p[i, 2], 
                      d[0], d[1], d[2], 
                      length=0.1, normalize=True, color='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Poses with Quaternion")
    plt.legend()
    return ax

def SE3_to_pose(Rt):
    return list(Rt.t) + list(R_to_quat(Rt.R))

def pose_to_SE3(pose):
    se3 = SE3.Tx(0)
    se3.R[:] = quat_to_R(pose[3:])
    se3.t = pose[:3]
    return se3


def xyzw2wxyz(q):
    return [q[3], q[0], q[1], q[2]]

def wxyz2xyzw(q):
    return [q[1], q[2], q[3], q[0]]

def R_to_quat(R):
    import transforms3d.quaternions as tq
    # return wxyz2xyzw(tq.mat2quat(R))
    return Rotation.from_matrix(R).as_quat()

def quat_to_R(q):
    """
    quat: x,y,z,w
    """

    import transforms3d.quaternions as tq
    # return tq.quat2mat(xyzw2wxyz(q))
    return  Rotation.from_quat(q).as_matrix()

def pose_to_Rt(pose):
    R = quat_to_R(pose[3:])
    t = np.array(pose[:3])
    return R, t

def Rt_to_pose(R, t):
    pose = list(t) + list(R_to_quat(R))
    return np.array(pose)


def Rt_to_SE3(R, t):
    pose = Rt_to_pose(R, t)
    return pose_to_SE3(pose)

def inverse_Rt(R, t):
    return R.T, -R.T @ t

def pose_to_T(pose):
    R, t = pose_to_Rt(pose)
    return Rt_to_T(R, t)

def Rt_to_T(R, t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.flatten()
    return T

def T_to_pose(T):
    t = T[:3, 3]
    R = T[:3, :3]
    q = R_to_quat(R)
    return np.concatenate((t, q))

def Ts_to_poses(Ts):
    poses = []
    for T in Ts:
        pose = T_to_pose(T).tolist()
        poses.append(pose)
    return poses

def poses_to_Ts(poses):
    Ts = []
    for pose in poses:
        T = pose_to_T(pose)
        Ts.append(T)
    Ts = np.array(Ts)
    return Ts



def inverse_pose(pose):
    R, t = pose_to_Rt(pose)
    R_star, t_star = inverse_Rt(R, t)
    return Rt_to_pose(R_star, t_star)

def Rt_dot(R1, t1, R2, t2):
    #R2 = R1 dot R0
    R = np.dot(R1, R2)
    t = np.dot(R1, t2) + t1
    return R, t

def relative_rotation(q1, q2):
    # q1 to q2
    # Convert quaternions to rotation objects

    rot1 = quat_to_R(q1)
    rot2 = quat_to_R(q2)

    # Compute the relative rotation from rot1 to rot2
    relative_rotation = rot2 * rot1.inv()
    return relative_rotation.as_matrix()

def transform_pose(R, t, pose):
    pose_star = pose.copy()
    pose_star[:3] = np.dot(R, pose[:3])+t
    if len(pose)>3:
        pose_star[3:] =  R_to_quat(R @ quat_to_R(pose[3:]))
    return pose_star
 
def transform_poses(R, t, poses):
    # single vector
    poses = np.array(poses)
    if len(poses.shape)<2:
        return transform_pose(R, t, pose)
    transformed_poses = []
    for pose in poses:
        transformed_pose = transform_pose(R, t, pose)
        transformed_poses.append(transformed_pose)
    transformed_poses = np.vstack(transformed_poses)
    return transformed_poses

def append_vector(matrix, vector_to_append):
    assert len(matrix.shape) == 2, "Input matrix must be 2D"
    res = []
    for row in matrix:
        res.append(list(row)+list(vector_to_append))

    return np.array(res)

# make sure right handed coordintate
def find_transformation(X, Y):
    """
    from X to Y
    Inputs: X, Y: lists of 3D points
    Outputs: R - 3x3 rotation matrix, t - 3-dim translation array.
    Find transformation given two sets of correspondences between 3D points.
    """
    # Calculate centroids
    cX = np.mean(X, axis=0)
    cY = np.mean(Y, axis=0)
    # Subtract centroids to obtain centered sets of points
    Xc = X - cX
    Yc = Y - cY

    # Calculate covariance matrix
    C = np.dot(Xc.T, Yc)
    # Compute SVD
    U, S, Vt = np.linalg.svd(C)
    # Determine rotation matrix
    R = np.dot(Vt.T, U.T)
    # Ensure a right-handed coordinate system
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)
    # Determine translation vector
    t = cY - np.dot(R, cX)
    return R, t

def compute_rigid_transform(source_points, target_points, enable_R=True):
    # Ensure inputs are numpy arrays
    source_points = np.array(source_points)
    target_points = np.array(target_points)

    # Compute centroids of each point set
    centroid_src = np.mean(source_points, axis=0)
    centroid_tgt = np.mean(target_points, axis=0)

    if enable_R:
        # Center the point clouds
        src_centered = source_points - centroid_src
        tgt_centered = target_points - centroid_tgt

        # Compute the covariance matrix
        H = np.dot(src_centered.T, tgt_centered)

        # Perform Singular Value Decomposition
        from scipy.linalg import svd
        U, _, Vt = svd(H)
        R = np.dot(Vt.T, U.T)
    else:
        R = np.eye(3)
    # Compute the translation vector
    t = centroid_tgt - np.dot(R, centroid_src)
    return R, t


def vec2mat(vec):
    if len(vec.shape)<2:
        vec = np.array([vec])   
    return vec

def poses_error(poses1, poses2):
    poses1 = vec2mat(poses1)
    poses2 = vec2mat(poses2)
    return np.mean(np.linalg.norm(poses1[:, :3] - poses2[:, :3], axis=1))


def deg2rad(deg):
    return [math.radians(degree) for degree in deg]

def rad2deg(radians):
    # Convert radians to degrees for each joint value
    return [math.degrees(rad) for rad in radians]

def swap_order(i, j, k):
    i[j], i[k] = i[k], i[j]
    return i

def reverse_sign(i, j):
    i[j] = -i[j]
    return i

def euler6d_to_se3(euler_6d):
    t = euler_6d[:3]
    r = euler_6d[3:]
    se3_object = SE3.Trans(t) * SE3.RPY(r, unit='rad')
    return se3_object

def se3_to_euler6d(se3_object):
    translation = se3_object.t
    rotation = se3_object.rpy(unit='rad')
    euler_6d = list(translation) + list(rotation)
    return euler_6d

def pose_to_euler6d(pose):
    se3_object = pose_to_SE3(pose)
    return se3_to_euler6d(se3_object)

def se3_to_str(se3, deg=True):
    euler6d = se3_to_euler6d(se3)
    t = euler6d[:3]
    r = euler6d[3:]
    if deg:
        r = np.degrees(r)
        rstr = "deg"
    else:
        rstr = "rad"
    return f"xyz(meter): {t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f},  rpy({rstr}): {r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}"

def same_position(pose1, pose2,  t_threshold=0.02, rad_threshold=0.4):
    """
    Compare two poses to determine if they are effectively the same position.
    
    Args:
        pose1, pose2: Marker poses to compare
        angle_threshold: Maximum angle difference in radians (default: 0.1)
        translation_threshold: Maximum translation difference in meters (default: 0.01)
    
    Returns:
        bool: True if poses are considered same, False otherwise
    """
    if pose1 is None and pose2 is None:
        return True
    if pose1 is None or pose2 is None:
        return False
    R_diff = np.dot(pose1.R.T, pose2.R)
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    translation_norm = np.linalg.norm(pose1.t - pose2.t)
    return angle < rad_threshold and translation_norm < t_threshold



if __name__=="__main__":
    folder = 'slam_data/0613-slam-aruco'
    joints_traj = np.load(f'{folder}/traj.npy')
    slam_poses = np.load(f'{folder}/slam_poses.npy')
    robot_poses = np.load(f'{folder}/robot_poses.npy')

    visualize_poses(robot_poses)
    plt.show()