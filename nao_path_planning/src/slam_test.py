#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import cv2
import numpy as np
import threading
import Queue as queue
import time
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
from scipy.optimize import least_squares


class KeyFrame:
    """
    Represents a keyframe. A keyframe is selected when the camera moves sufficiently.
    It stores the grayscale image, extracted ORB features,descriptors, and the global pose.
"""    
    def __init__(self, id, image, pose, keypoints, descriptors):
        self.id = id
        self.image = image            # Grayscale image
        self.pose = pose              # 4x4 transformation matrix (global pose)
        self.keypoints = keypoints    # ORB keypoints
        self.descriptors = descriptors  # ORB descriptors
        self.map_points = []          # (In a full system, triangulated 3D points)
        self.connected_keyframes = {} # Covisibility graph: keyframe_id -> weight

class Map:    
    # The global map stores keyframes and (optionally) triangulated map points.    
    def __init__(self):
        self.keyframes = []  # List of keyframes
        self.map_points = [] # Global 3D map points

    def add_keyframe(self, kf):
        self.keyframes.append(kf)

    def add_map_point(self, pt):
        self.map_points.append(pt)

#VISUAL ODOMETRY        
class VisualOdometry:    
    #ORB feature extraction, matching, and pose estimation.    
    def __init__(self, calib_file=None):
        self.orb = cv2.ORB_create(3000)
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        if calib_file is not None:
            self.K, self.P = self._load_calib(calib_file)
        else:
            # Dummy calibration (fx, fy, cx, cy) – adjust these values for your camera.
            self.K = np.array([[700, 0, 320],
                               [0, 700, 240],
                               [0,   0,   1]], dtype=np.float64)
            self.P = np.hstack((self.K, np.zeros((3, 1))))
    
    def _load_calib(self, filepath):
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P

    def process_frame(self, prev_frame, curr_frame):
        """
        Extracts features from 2 frames, matches them, and computes the Essential matrix to recover motion.
        Returns a 4x4 transformation matrix, keypoints, descriptors, and matched points.
        """
        kp1, des1 = self.orb.detectAndCompute(prev_frame, None)
        kp2, des2 = self.orb.detectAndCompute(curr_frame, None)
        if des1 is None or des2 is None:
            return None, None, None, None
        matches = self.flann.knnMatch(des1, des2, k=2)
        good = []
        for match in matches:
            if len(match) < 2:
                continue
            m = match[0]
            n = match[1]
            if m.distance < 0.8 * n.distance:
                good.append(m)
        if len(good) < 8:
            return None, None, None, None
        pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
        E, mask = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return None, None, None, None
        retval, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        return T, kp2, des2, pts2

#TRACKER          
class Tracker:
    # Processes frames, computes relative motion, updates global pose and creates keyframes when sufficient motion is detected.
    def __init__(self, vo, slam_map, keyframe_queue):
        self.vo = vo
        self.map = slam_map
        self.keyframe_queue = keyframe_queue
        self.prev_frame = None
        self.global_pose = np.eye(4)  # Global pose initialized to identity.
        self.last_keyframe_pose = np.eye(4)
        self.frame_id = 0
        self.keyframe_threshold_translation = 0.02  # Threshold (in meters).

    def track(self, frame):
        # If frame is not already grayscale, convert it.
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame.copy()
        if self.prev_frame is None:
            self.prev_frame = gray
            # Create the very first keyframe.
            kp, des = self.vo.orb.detectAndCompute(gray, None)
            kf = KeyFrame(self.frame_id, gray, self.global_pose.copy(), kp, des)
            self.map.add_keyframe(kf)
            self.keyframe_queue.put(kf)
            self.last_keyframe_pose = self.global_pose.copy()
            self.frame_id += 1
            return self.global_pose
        T, kp, des, pts = self.vo.process_frame(self.prev_frame, gray)
        if T is None:
            return self.global_pose
        self.global_pose = np.dot(self.global_pose, np.linalg.inv(T))
        delta = np.linalg.norm(self.global_pose[:3, 3] - self.last_keyframe_pose[:3, 3])
        if delta > self.keyframe_threshold_translation:
            kf = KeyFrame(self.frame_id, gray, self.global_pose.copy(), kp, des)
            self.map.add_keyframe(kf)
            self.keyframe_queue.put(kf)
            self.last_keyframe_pose = self.global_pose.copy()
            self.frame_id += 1
        self.prev_frame = gray
        return self.global_pose

# LOCAL MAPPER      
class LocalMapper(threading.Thread):
    """
    Runs in a separate thread to process new keyframes and run local bundle adjustment.
    """
    def __init__(self, slam_map, keyframe_queue):
        super(LocalMapper, self).__init__()
        self.map = slam_map
        self.queue = keyframe_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                kf = self.queue.get(timeout=1)
                print("[LocalMapper] Processing keyframe {}".format(kf.id))
                self.bundle_adjustment([kf])
            except queue.Empty:
                continue

    def bundle_adjustment(self, local_keyframes):
        # Placeholder for local bundle adjustment.
        print("[LocalMapper] Running local bundle adjustment on keyframes:", [kf.id for kf in local_keyframes])

    def stop(self):
        self.running = False

#LOOP CLOSURE
class LoopClosure(threading.Thread):
    """
    Simulates loop closure detection and global pose graph optimization.In a real system, you'd use a nearest-neighbor or bag-of-words method.
    """
    def __init__(self, slam_map):
        super(LoopClosure, self).__init__()
        self.map = slam_map
        self.running = True

    def run(self):
        while self.running:
            if len(self.map.keyframes) > 10:
                print("[LoopClosure] Detected potential loop closure. Running global optimization...")
                self.global_optimization()
            time.sleep(5)

    def global_optimization(self):
        # Placeholder for global pose graph optimization.
        print("[LoopClosure] Running global optimization on the full map.")

    def stop(self):
        self.running = False

# TRIANGULATION FUNCTIONS 
def triangulate_keyframe_pair(kf1, kf2, K):
    """
    Matches features between two keyframes, triangulates the matching points using the
    projection matrices derived from the keyframe poses, and returns 3D points that are in front of both cameras.
    """
    # Use BFMatcher with Hamming distance (for ORB).
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(kf1.descriptors, kf2.descriptors)
    if len(matches) == 0:
        return np.empty((0, 3))
    matches = sorted(matches, key=lambda x: x.distance)
    pts1 = []
    pts2 = []
    for m in matches:
        pts1.append(kf1.keypoints[m.queryIdx].pt)
        pts2.append(kf2.keypoints[m.trainIdx].pt)
    pts1 = np.float32(pts1)
    pts2 = np.float32(pts2)
    if pts1.shape[0] < 8:
        return np.empty((0, 3))
    # Compute projection matrices:
    # Assuming keyframe.pose is camera-to-world, then the world-to-camera transform is its inverse.
    T1_inv = np.linalg.inv(kf1.pose)
    T2_inv = np.linalg.inv(kf2.pose)
    P1 = np.dot(K, T1_inv[:3, :])
    P2 = np.dot(K, T2_inv[:3, :])
    pts4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
    pts3d = pts4d[:3, :] / pts4d[3, :]
    pts3d = pts3d.T
    # Filter: keep points with positive z in both camera frames.
    valid_points = []
    for X in pts3d:
        X_hom = np.append(X, 1.0)
        X1 = np.dot(T1_inv, X_hom)
        X2 = np.dot(T2_inv, X_hom)
        if X1[2] > 0 and X2[2] > 0:
            valid_points.append(X)
    if valid_points:
        return np.array(valid_points)
    else:
        return np.empty((0, 3))

def visualize_point_cloud(points):
    
#Visualizes a 3D point cloud using matplotlib.
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if points.shape[0] > 0:
        ax.scatter(points[:,0], points[:,1], points[:,2], c='g', marker='.', s=5)
    ax.set_title("Triangulated 3D Point Cloud")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()


#VISUALIZATION  
def visualize_2d_trajectory(trajectory):
    plt.figure()
    plt.plot(trajectory[:,0], trajectory[:,2], '-o', markersize=2)
    plt.title("Camera Trajectory (X-Z Projection)")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.grid(True)
    plt.show()

def visualize_3d_graph(slam_map):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = [], [], []
    for kf in slam_map.keyframes:
        pos = kf.pose[:3, 3]
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])
    ax.scatter(xs, ys, zs, c='r', marker='o')
    ax.plot(xs, ys, zs, c='b')  # Connect consecutive keyframes.
    ax.set_title("3D Keyframe Trajectory / Covisibility Graph")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()
#main
def main():
    parser = argparse.ArgumentParser(description="Live SLAM for NAO - Mapping and Visualization (Python 2.7)")
    parser.add_argument("--mode", type=str, default="nao", help="Mode: 'nao' for NAO robot, 'laptop' for webcam")
    parser.add_argument("--calib", type=str, default=None, help="Path to calibration file (if available)")
    args = parser.parse_args()

    # Initialize components.
    vo = VisualOdometry(args.calib)
    slam_map = Map()
    keyframe_queue = queue.Queue()
    tracker = Tracker(vo, slam_map, keyframe_queue)
    local_mapper = LocalMapper(slam_map, keyframe_queue)
    loop_closure = LoopClosure(slam_map)

    # Start background threads.
    local_mapper.start()
    loop_closure.start()

    trajectory = []  # To store global camera positions.

    if args.mode.lower() == "nao":
        #NAO ROBOT MODE
        from naoqi import ALProxy
        ip = "169.254.206.233" #"192.168.0.108"  # Replace with your NAO's IP.
        port = 9559
        videoProxy = ALProxy("ALVideoDevice", ip, port)
        resolution = 2   # Example: kQVGA (320x240)
        colorSpace = 8   # Grayscale
        fps = 30
        videoClient = videoProxy.subscribeCamera("python_client", 0, resolution, colorSpace, fps)
        print("NAO live feed started. Press 'q' in the OpenCV window to terminate.")
        while True:
            image_remote = videoProxy.getImageRemote(videoClient)
            if image_remote is None:
                continue
            width = image_remote[0]
            height = image_remote[1]
            frame = np.frombuffer(image_remote[6], dtype=np.uint8).reshape((height, width))
            pose = tracker.track(frame)
            trajectory.append(pose[:3, 3].copy())
            cv2.imshow("NAO Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Terminating NAO live feed...")
                break
        videoProxy.unsubscribe(videoClient)
    else:
        # LAPTOP/WEBCAM MODE for testing
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open webcam")
            return
        print("Webcam live feed started. Press 'q' to terminate.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            pose = tracker.track(frame)
            trajectory.append(pose[:3, 3].copy())
            cv2.imshow("Webcam Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Terminating webcam live feed...")
                break
        cap.release()
    cv2.destroyAllWindows()

    # Stop background threads.
    local_mapper.stop()
    loop_closure.stop()
    local_mapper.join()
    loop_closure.join()

    # Convert trajectory list to numpy array.
    if len(trajectory) > 0:
        traj_arr = np.array(trajectory)
        visualize_2d_trajectory(traj_arr)

    # Visualize 3D keyframe trajectory.
    if len(slam_map.keyframes) > 0:
        visualize_3d_graph(slam_map)

    # Triangulate a pt. cloud from consecutive keyframe pairs
    all_points = []
    keyframes = slam_map.keyframes
    if len(keyframes) >= 2:
        for i in range(len(keyframes)-1):
            pts3d = triangulate_keyframe_pair(keyframes[i], keyframes[i+1], vo.K)
            if pts3d.shape[0] > 0:
                all_points.append(pts3d)
    if all_points:
        point_cloud = np.vstack(all_points)
        visualize_point_cloud(point_cloud)
    else:
        print("No valid 3D points were triangulated.")

if __name__ == "__main__":
    main()