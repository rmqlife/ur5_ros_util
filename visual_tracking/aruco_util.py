import cv2
import cv2.aruco as aruco
import numpy as np
from myPlanner import Rt_to_pose

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


if __name__ == "__main__":
    generate_markers(start_id=0, end_id=20, size=100, folder="arucos-4x4-100")

    