#!/usr/bin/env python3

"""
Test script for LightGlue feature matching between two images.
This script demonstrates keypoint matching and 3D pose estimation using LightGlue.
"""

import cv2
from myGlue import MyGlue, replace_rgb_to_depth
from mySensor import load_intrinsics

def main():
    """Main function to run the LightGlue matching demo."""
    # Initialize LightGlue matcher
    glue = MyGlue(match_type="Aruco")

    home = 'slam_data/0613-slam-aruco'
    image_path1 = f"./test_data/camera/camera_rgb_0.png"
    image_path2 = f"./test_data/camera/camera_rgb_1.png"
    depth_path1 = replace_rgb_to_depth(image_path1)
    depth_path2 = replace_rgb_to_depth(image_path2)

    # Load RGB images
    rgb1 = cv2.imread(image_path1)
    rgb2 = cv2.imread(image_path2)
    if rgb1 is None or rgb2 is None:
        raise FileNotFoundError("Failed to load RGB images")

    # Match keypoints
    pts0, pts1 = glue.match(rgb1, rgb2)

    # Visualize matches
    show_frame = rgb1.copy()
    for (x1, y1), (x2, y2) in zip(pts0, pts1):
        cv2.line(show_frame, (int(x1), int(y1)), 
                (int(x2), int(y2)), (255, 255, 0), 2)


    depth1 = cv2.imread(depth_path1, cv2.IMREAD_UNCHANGED)
    depth2 = cv2.imread(depth_path2, cv2.IMREAD_UNCHANGED)
    if depth1 is None or depth2 is None:
        raise FileNotFoundError("Failed to load depth images")

    # Estimate 3D transformation
    intrinsics = load_intrinsics("../config/camera_intrinsics.json")
    R, t = glue.match_3d(pts0, pts1, depth1, depth2, intrinsics)
    print("Rotation matrix:\n", R)
    print("Translation vector:\n", t)

    # Display results
    cv2.imshow('Matches Visualization', show_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

