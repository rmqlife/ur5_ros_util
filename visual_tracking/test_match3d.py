import cv2
import numpy as np
from lightglue_util import MyGlue, load_intrinsics
from myPlanner.pose_util import pose_to_SE3, Rt_to_pose

home = 'data/test_pose'
id_src = 0
id_dst = 2
intrinsics_path="slam_data/intrinsics_d435.json"

def load_rgb_depth(img_id):
    image_path = f"{home}/rgb_{img_id}.png"
    depth_path = f"{home}/depth_{img_id}.png"  # Assuming depth images have a different naming convention
    rgb = cv2.imread(image_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    return rgb, depth    

if __name__ == "__main__":
    glue = MyGlue(match_type="Aruco") # Aruco LightGlue
    
    # Fixed the variable names to match the function definition
    src_rgb, src_depth = load_rgb_depth(id_src)
    dst_rgb, dst_depth = load_rgb_depth(id_dst)  # Assuming you want to load the second image as the destination

    intrinsics = load_intrinsics(intrinsics_path)
    # Fixed the variable names to match the loaded images
    src_pts, dst_pts = glue.match(src_rgb, dst_rgb)
    print(f"src {src_pts}, dst {dst_pts}")
    R, t = glue.match_3d(src_pts, dst_pts, src_depth, dst_depth, intrinsics, show=False)
    pose_delta_0 = pose_to_SE3(Rt_to_pose(R,t))
    print(pose_delta_0.printline())

    
    from follow_aruco import get_marker_pose
    src_pose, corners = get_marker_pose(src_rgb)
    dst_pose, corners = get_marker_pose(dst_rgb)
    pose_delta = dst_pose * src_pose.inv()
    pose_delta.printline()