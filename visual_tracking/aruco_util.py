import cv2
import cv2.aruco as aruco
import numpy as np
from myPlanner import Rt_to_pose
import matplotlib.pyplot as plt
import math

ARUCO_DICT_NAME = aruco.DICT_4X4_100
my_aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)

# Function to detect ArUco markers

def detect_aruco(image, draw_flag=False):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = aruco.ArucoDetector(my_aruco_dict, aruco.DetectorParameters())
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    if corners is None or ids is None:
        return [], []
    else:
        # Draw detected markers on the image
        if draw_flag and ids is not None:
            image = aruco.drawDetectedMarkers(image, corners, ids)
        corners = [c[0] for c in corners]
        ids = [c[0] for c in ids]
        return corners, ids
    
def draw_marker_frames(image, corners, poses, intrinsics, marker_size):
    for i, (corner, (R, tvec)) in enumerate(zip(corners, poses)):
        if R is not None:
            # Define the 3D axes
            axis = np.float32([[0, 0, 0], [0, marker_size, 0], [marker_size, marker_size, 0], [marker_size, 0, 0]]).reshape(-1, 3)
            # Project 3D points to 2D image plane
            axis_img, _ = cv2.projectPoints(axis, R, tvec, intrinsics, np.zeros((5, 1)))
            axis_img = axis_img.reshape(-1, 2)
            corner = np.int32(corner).reshape(-1, 2)
            # Draw the marker's axis
            image = cv2.drawContours(image, [np.int32(axis_img)], -1, (0, 255, 0), 2)
            image = cv2.polylines(image, [np.int32(axis_img[:2])], False, (0, 0, 255), 2)  # Draw lines

    return image


# Function to generate ArUco markers
def generate_aruco_marker(marker_id, marker_size, output_file):
    # Generate ArUco marker image
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = aruco.generateImageMarker(my_aruco_dict, marker_id, marker_size, marker_image, 1)
    cv2.imwrite(output_file, marker_image)


def estimate_marker_pose(corner, marker_size, intrinsics):
    return estimate_markers_poses([corner], marker_size, intrinsics)[0]


def estimate_markers_poses(corners, marker_size, intrinsics):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    '''
    # make sure the aruco's orientation in the camera view! 
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    mtx = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                    [0, intrinsics["fy"], intrinsics["cy"]],
                    [0, 0, 1]], dtype=np.float32)
    distortion = np.zeros((5, 1))  # Assuming no distortion
    # distortion  = np.array([[ 0.00377581 , 0.00568285 ,-0.00188039, -0.00102468 , 0.02337337]])
    poses = []
    for c in corners:
        ret, rvec, tvec = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_EPNP)
        if ret:
            tvec = tvec.reshape((3,))
            R, _ = cv2.Rodrigues(rvec)
            pose = Rt_to_pose(R, tvec)  # Ensure Rt_to_pose is correctly implemented
            poses.append(pose)
        else:
            print("Pose estimation failed for one of the markers")
    return poses


def generate_markers(start_id=0, end_id=20, size=100, folder="arucos-new"):
    """Generate a range of ArUco markers"""
    from os.path import join as pathjoin
    import os

    os.makedirs(folder, exist_ok=True)
    try:
        for marker_id in range(start_id, end_id):
            output_file = pathjoin(folder,f'aruco_marker_{marker_id}.png')
            generate_aruco_marker(marker_id, size, output_file)
            print(f"Generated marker {marker_id}")
    except Exception as e:
        print(f"Error generating markers: {str(e)}")



def project_to_3d(points, depth, intrinsics, show=False):
    if show:
        plt.imshow(depth)
    
    points_3d = list()
    for x,y in points:
        x = math.floor(x) 
        y = math.floor(y)
        d = depth[y][x]        
        # Plot points (x, y) on the image
        if show:
            if d>0:
                plt.scatter(x, y, color='blue', s=10)  # Adjust the size (s) as needed
            else:
                plt.scatter(x, y, color='red', s=10)
        # z = d / depth_scale
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        # 3d point in meter
        z = d / 1000
        x = (x - intrinsics['cx']) * z / intrinsics['fx'] 
        y = (y - intrinsics['cy']) * z / intrinsics['fy'] 
        
        if show:
            print(f'x:{x} \t y:{y} \t z:{z}')
        points_3d.append((x,y,z))
        
    if show:
        plt.axis('off')  # Turn off axis labels
        plt.show()
    
    return points_3d
    

def get_aruco_poses(frame, depth, camera_intrinsics,  default_marker_size, draw_flag=True, verbose=False):
    poses_dict = {}
    # make sure the aruco's orientation in the camera view! 
    corners, ids = detect_aruco(frame, draw_flag=draw_flag)
    marker_size = default_marker_size
    for corner, id in zip(corners, ids):
        if depth is not None:
            points = project_to_3d(corner, depth, camera_intrinsics, show=False)
            if all(validate_point(point) for point in points):
                marker_size = average_distance(points)
        pose = estimate_marker_pose(corner, marker_size=marker_size, intrinsics=camera_intrinsics) 
        poses_dict[id] = pose
        if verbose:
            print(f"id: {id}, pose: {pose}, size: {marker_size}")
            print()
    return poses_dict


def validate_point(point):
    # if z almost is the zero, then the point is not valid
    if abs(point[2]) < 1e-4:
        return False
    return True

def average_distance(points):
    distances = []
    # compute 0 to 1, 1 to 2, 2 to 3, 3 to 0
    for i in range(len(points)):
        a1 = points[i]
        a2 = points[(i + 1) % len(points)]
        a1 = np.array(a1)
        a2 = np.array(a2)
        distance = np.linalg.norm(a1 - a2)
        distances.append(distance)
    mean_distance = np.mean(distances)
    return mean_distance


if __name__ == "__main__":
    generate_markers(start_id=0, end_id=20, size=100, folder="arucos-4x4-100")

    