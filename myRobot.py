#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import math

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

class MyRobot:
    def __init__(self):
        subscriber_topic = '/joint_states' 
        # if using '/scaled_pos_joint_traj_controller/state', the state is unreachable in passive mode
        self.sub = rospy.Subscriber(subscriber_topic, JointState, self.subscriber_callback)

        publisher_topic ='/scaled_pos_joint_traj_controller/command'
        self.pub = rospy.Publisher(publisher_topic, JointTrajectory, queue_size=1)

        # Wait for the subscriber to receive joint names
        rospy.sleep(0.5)

    def subscriber_callback(self, data):
        self.joint_positions = np.array(data.position)
        self.joint_names = data.name

        # if using '/joint_states' as subscriber, swap orders of elbow and should_pan
        swap_order(self.joint_positions, 0, 2)
        swap_order(self.joint_names, 0, 2)

        self.velocity = np.array(data.velocity)


    def get_joints(self):
        return self.joint_positions

    def move_joints_smooth(self, joints, coef=10, joint_thresh=1, wait=False):
        max_joint_movement = np.max(np.abs(joints - self.get_joints()))
        if max_joint_movement < joint_thresh:
            self.move_joints(joints, duration=coef*max_joint_movement, wait=wait)
            return False
        print('Joint movement exceeds joint threshold: ', joint_thresh)
        return True

    def move_joints(self, joints, duration=0.1, wait=True):
        # Create a JointTrajectory message
        joint_traj = JointTrajectory()

        # Set the joint names from the received message
        joint_traj.joint_names = self.joint_names

        # Create a JointTrajectoryPoint for the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = joints

        # Set the time from start for this point
        point.time_from_start = rospy.Duration(duration)

        # Append the JointTrajectoryPoint to the trajectory
        joint_traj.points.append(point)
        self.pub.publish(joint_traj)  # Call the publish method

        threshold = 1e-4
        if wait:
            # Check if joint positions have been reached
            while not rospy.is_shutdown():
                if np.all(np.abs(self.get_joints() - joints) < threshold):
                    break
                # print('wait', np.abs(current_positions - joint_positions))
                rospy.sleep(0.001)  # Adjust sleep duration as needed
        else:
            rospy.sleep(duration)


if __name__ == '__main__':
    rospy.init_node('my_robot_ns_node')
    robot = MyRobot()

    # Assuming you want to echo the joint values after a short delay
    rospy.sleep(1)  # Wait for the subscriber to receive initial data
    while True:
        current_joints = robot.get_joints()
        rospy.loginfo("Current Joint Values: %s", current_joints)
        rospy.sleep(1/20)