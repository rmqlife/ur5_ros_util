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

def linear_interpolate(start, end, num_points):
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        point = (1 - t) * start + t * end
        points.append(point)
    return points

def square_points(center, side_length, points_per_side):
    half_side = side_length / 2
    # Define the corners of the square
    corners = [
        np.array([center[0] - half_side, center[1] - half_side, center[2]]),  # Bottom-left
        np.array([center[0] + half_side, center[1] - half_side, center[2]]),  # Bottom-right
        np.array([center[0] + half_side, center[1] + half_side, center[2]]),  # Top-right
        np.array([center[0] - half_side, center[1] + half_side, center[2]])   # Top-left
    ]

    # interpolate points between corners
    points = []
    for i in range(len(corners)):
        start = corners[i]
        end = corners[(i + 1) % len(corners)]
        points.extend(linear_interpolate(start, end, num_points=points_per_side))
    return points

if __name__ == "__main__":
    dry_run = False
    rospy.init_node('test_coda', anonymous=False)
    robot = init_robot_with_ik()

    init_pose = robot.get_pose()
    print("init pose at")
    init_pose.printline()


    center = init_pose.t
    square_points = square_points(center, side_length=0.2, points_per_side=3)

    for point in square_points:
        pose = init_pose.copy()
        pose.t = point
        pose.printline()
        if not dry_run:
            robot.goto_pose(pose, wait=True, coef=3)
