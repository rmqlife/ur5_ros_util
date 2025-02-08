import os
import queue
import pickle
import time
import cv2
import numpy as np
from myGlue import filter_out_zeros_points
from aruco_util import detect_aruco, project_to_3d
from myPlanner.pose_util import Rt_to_SE3, find_transformation, pose_to_euler6d, se3_to_euler6d

def project_corners(corners, depth, intrinsics):
    corners = np.array(corners).reshape(-1, 2)
    pts = project_to_3d(corners, depth, intrinsics)
    pts = np.array(pts)
    pts = pts.reshape(-1,4,3)
    pts = [pt.tolist() for pt in pts]
    return pts

def build_aruco_state(frame, depth, intrinsics, robot_pose):
    aruco_corners, aruco_ids = detect_aruco(frame, draw_flag=True)
    if len(aruco_ids)>0:
        aruco_pts = project_corners(aruco_corners, depth, intrinsics)
        aruco_state = ArucoState(aruco_ids, aruco_pts, aruco_corners, robot_pose)        
        return aruco_state
    return None

def corners_to_pts(corners):
    pts = np.array(corners).reshape(-1, 2)
    pts = [pt.tolist() for pt in pts] 
    return pts


def draw_match_line(frame, state0, state1):
    ids0 = state0.ids
    ids1 = state1.ids

    common_ids = set(ids0) & set(ids1)
    common_ids = list(common_ids)
    if common_ids:
        # Extract poses for common IDs
        corners0 = [state0.corner(id) for id in common_ids]
        corners1 = [state1.corner(id) for id in common_ids]

        # reshape to 2d array
        corners0 = corners_to_pts(corners0)   
        corners1 = corners_to_pts(corners1)

        for (x1, y1), (x2, y2) in zip(corners0, corners1):
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,0), 2)

def match_aruco_state(state0, state1):
    if state0 is None or state1 is None:
        return None
    
    ids0 = state0.ids
    ids1 = state1.ids

    common_ids = set(ids0) & set(ids1)
    common_ids = list(common_ids)
    pts0 = []
    pts1 = []
    if common_ids:
        # Extract pots for common IDs
        for id in common_ids:
            pts0 += state0.pt(id)
            pts1 += state1.pt(id)
        # filter out zeros points
        pts0, pts1 = filter_out_zeros_points(pts0, pts1)
        
        R, t = find_transformation(pts0, pts1)
        action = Rt_to_SE3(R, t)
        return action
    return None

class ArucoState:
    def __init__(self, ids, pts, corners, robot_pose):
        self.ids = ids
        self.pts = {id: pt for id, pt in zip(ids, pts)}
        self.corners = {id: corner for id, corner in zip(ids, corners)}
        self.robot_pose = robot_pose
        self.timestamp = time.strftime("%Y%m%d-%H%M%S")

    def pt(self, id):
        return self.pts[id]
    
    def corner(self, id):
        return self.corners[id]
    
    def set_type(self, type):
        self.type = type

    def __gt__(self, other):
        return self.timestamp > other.timestamp

    def robot_pose_equal(self, other):
        if self.robot_pose_dist(other) < 0.05:
            return True
        return False
    
    def robot_pose_dist(self, other):
        e1 = np.array(pose_to_euler6d(self.robot_pose)) 
        e2 = np.array(pose_to_euler6d(other.robot_pose))
        t_norm = np.linalg.norm(e1[:3]-e2[:3])
        r_norm = np.linalg.norm(e1[3:]-e2[3:])
        return t_norm + r_norm
    
def find_next(states, current):
    for state in states:
        if state > current:
            return state
    if len(states)>0:
        return states[0]
    return None

def action_norm(action):
    norm = np.array(se3_to_euler6d(action)) 
    return np.linalg.norm(norm[:3]) + np.linalg.norm(norm[3:])

class MyArucoStates:
    def __init__(self, filename):
        self.filename = filename
        self.action_queue = queue.Queue()
        self.states = []
        if os.path.exists(filename):
            with open(filename, "rb") as f:
                self.states = pickle.load(f)
        
    def add_state(self, state):
        self.states.append(state)

    def save(self):
        with open(self.filename, "wb") as f:
            pickle.dump(self.states, f)

    def next_state(self, state):
        if len(self.states) == 0:  
            return None
        
        if state is None:
            return self.states[0]
        return find_next(self.states, state)
    
    def match(self, state):
    # you should build the graph of states to find the nearest state
    # based on the goal state, so only 1 closest and latest state is needed to be stored
    
        min_dist = 1000
        for h_state in self.states:
            action = match_aruco_state(state, h_state)
            if action is not None:
                # find a nearest state
                dist = action_norm(action)
            else:
                dist = 1000
            # find a nearest state
            if dist < min_dist:
                min_dist = dist
                min_state = h_state
        if min_dist < 1000:
            return min_state
        return None
    
    # last state
    def last_state(self):
        if len(self.states) == 0:
            return None
        return self.states[-1]
    
    def push_back(self, state):
        if len(self.states) > 0:
            if self.states[-1].robot_pose_equal(state):
                return -1
        self.states.append(state)
        return len(self.states)-1
if __name__ == "__main__":
    filename = "./data_states/recorded_arucos_0208/goal_states.pkl"
    states = MyArucoStates(filename)
    
    current_goal = None
    for i in range(10):
        current_goal = states.next_state(current_goal)
        print(current_goal.timestamp)


    
