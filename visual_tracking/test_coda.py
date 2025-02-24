from myPlanner.pose_util import *
from myPlanner import init_robot_with_ik
import rospy

def circle_points(center, radius=0.1, num_points=20):
    points = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        point = np.array([center[0] + delta_x, center[1] + delta_y, center[2]])
        points.append(point)

    return points

def square_points(center, side_length=0.1, num_points_per_side=0):
    points = []
    half_side = side_length / 2

    # Define the corners of the square
    corners = [
        np.array([center[0] - half_side, center[1] - half_side, center[2]]),  # Bottom-left
        np.array([center[0] + half_side, center[1] - half_side, center[2]]),  # Bottom-right
        np.array([center[0] + half_side, center[1] + half_side, center[2]]),  # Top-right
        np.array([center[0] - half_side, center[1] + half_side, center[2]])   # Top-left
    ]

    # Interpolate points between corners
    for i in range(4):
        start = corners[i]
        end = corners[(i + 1) % 4]
        for j in range(num_points_per_side):
            t = j / (num_points_per_side - 1)
            point = (1 - t) * start + t * end
            points.append(point)

    return points

if __name__ == "__main__":
    dry_run = True
    rospy.init_node('test_move', anonymous=False)
    robot = init_robot_with_ik()

    init_pose = robot.get_pose()
    print("init pose at", init_pose)

    center = init_pose[:3]
    square_points = square_points(center, side_length=0.2, num_points_per_side=0)

    for point in square_points:
        pose = np.concatenate([point, init_pose[3:]])
        pose = pose_to_SE3(pose)
        pose.printline()
        if not dry_run:
            robot.goto_pose(pose, dry_run=False, coef=3)
