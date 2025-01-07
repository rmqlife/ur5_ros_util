import cv2
from aruco_util import detect_aruco, estimate_markers_poses, generate_aruco_marker
from mySensor import load_intrinsics

def test_on_image(image_path='./aruco_test.jpg'):
    try:
        # Load the image
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("Image not found or unable to load.")
        
        # Load camera intrinsics
        intrinsics = load_intrinsics("../config/camera_intrinsics.json")
        print(f"Image shape: {image.shape}")

        # Detect ArUco markers
        corners, ids = detect_aruco(image, draw_flag=True)
        
        if ids is not None and len(ids) > 0:
            print(f'Detected markers IDs: {ids}')
            print(f'Corner points: {corners}')
            poses = estimate_markers_poses(corners, marker_size=0.03, intrinsics=intrinsics)
            print('Marker poses:', poses)
        else:
            print('No ArUco markers detected in the image.')

        # Display results
        cv2.imshow('ArUco Detection', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error: {str(e)}")



if __name__ == "__main__":
    test_on_image()