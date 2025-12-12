import cv2
import numpy as np
import time
import threading
import heapq
from naoqi import ALProxy

# Set NAO Robot Connection Details
IP = "192.168.0.108"  # Replace with your NAO robot's IP address
PORT = 9559

# Define Tennis Court Grid 
court_grid = np.zeros((10, 20))  # 0 = Free space, 1 = Obstacle

# Initialize NAOqi Proxies
try:
    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    video = ALProxy("ALVideoDevice", IP, PORT)
except Exception as e:
    print("Could not connect to NAO:", e)
    exit(1)

# Wake up and set initial posture
motion.wakeUp()
posture.goToPosture("StandInit", 0.5)

# A* Path Planning with Obstacle Avoidance
class Node:
    def __init__(self, x, y, cost=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0, Node(*start)))
    visited = set()

    while open_list:
        _, current = heapq.heappop(open_list)

        if (current.x, current.y) == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]  # Reverse path

        if (current.x, current.y) in visited:
            continue
        visited.add((current.x, current.y))

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:  # Move directions
            nx, ny = current.x + dx, current.y + dy

            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                new_cost = current.cost + 1 + heuristic((nx, ny), goal)
                heapq.heappush(open_list, (new_cost, Node(nx, ny, new_cost, current)))

    return None  # No path found

# Global Variables for Ball Tracking
ball_info_global = None
frame_width_global = None
ball_lock = threading.Lock()
stop_event = threading.Event()

# State variables for motion control
state_lock = threading.Lock()
state = "SEARCHING"  # Start in SEARCHING state
lost_time = None  # Time when ball was lost

def detect_ball(frame):
    """
    Detects a tennis ball in the given frame using HSV thresholding and contour detection.
    Returns the annotated frame and ball info (coordinates and radius).
    """
    # Expand color detection ranges
    lower_yellow = np.array([20, 80, 80])   # Wider lower bound
    upper_yellow = np.array([40, 255, 255])  # Wider upper bound
    
    # Multiple filtering steps
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ball_info = None
    
    if contours:
        # Filter contours by area and aspect ratio
        valid_contours = [c for c in contours if (cv2.contourArea(c) > 50) and (cv2.contourArea(c) < 5000)]
        
        if valid_contours:
            c = max(valid_contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.putText(frame, "Ball", (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            ball_info = {"x": int(x), "y": int(y), "radius": int(radius)}
    return frame, ball_info

# Video Capture Thread
def video_capture_thread():
    global ball_info_global, frame_width_global
    while not stop_event.is_set():
        frame = get_frame_from_nao(IP, PORT)
        if frame is None:
            continue
        
        annotated_frame, detected_ball = detect_ball(frame)

        with ball_lock:
            if detected_ball:
                ball_info_global = detected_ball
            else:
                ball_info_global = None
            frame_width_global = frame.shape[1]

        cv2.imshow("Ball Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            stop_event.set()
            break

        time.sleep(0.03)  # Smooth capture

# Ball Tracking and Motion Control
def motion_control_thread():
    global state, lost_time
    lost_threshold = 1.0  # seconds before switching to SEARCHING

    while not stop_event.is_set():
        with ball_lock:
            current_ball = ball_info_global
            current_frame_width = frame_width_global

        with state_lock:
            if current_ball is not None:
                if state == "SEARCHING":
                    print("Ball found! Switching to TRACKING mode.")
                state = "TRACKING"
                lost_time = None
            else:
                if state == "TRACKING":
                    if lost_time is None:
                        lost_time = time.time()
                    elif time.time() - lost_time > lost_threshold:
                        print("Ball lost! Switching to SEARCHING mode.")
                        state = "SEARCHING"

        if state == "TRACKING" and current_ball is not None:
            move_to_ball(current_ball, current_frame_width, motion)
        elif state == "SEARCHING":
            search_for_ball(motion)

        time.sleep(0.1)  # Control loop delay

# Move to Detected Ball
def move_to_ball(ball_info, frame_width, motion_proxy):
    # Proportional control
    ball_x = ball_info["x"]
    frame_center = frame_width / 2
    
    # Calculate error and control signals
    error_x = ball_x - frame_center
    rotation_gain = 0.005  # Adjustable gain
    forward_gain = 0.02    # Adjustable gain
    
    # Adaptive movement based on ball size and position
    rotation = -rotation_gain * error_x
    forward = forward_gain * (100 / ball_info["radius"])  # Closer ball = slower approach
    
    print("TRACKING: Forward={:.3f}, Rotation={:.3f}".format(forward, rotation))
    motion_proxy.moveToward(forward, 0.0, rotation)

# Search Behavior When Ball is Lost
def search_for_ball(motion_proxy):
    # More dynamic search pattern
    search_patterns = [
        (0, 0, np.pi/4),   # Rotate 45 degrees
        (0, 0, -np.pi/4),  # Rotate -45 degrees
        (0.2, 0, 0),       # Small forward step
        (0, 0, np.pi/2)    # Rotate 90 degrees
    ]
    
    for step in search_patterns:
        if ball_info_global is not None:
            return  # Stop if ball detected
        
        motion_proxy.moveTo(*step)
        time.sleep(0.5)  # Processing time

# Get Frame from NAO Camera
def get_frame_from_nao(ip, port):
    videoProxy = ALProxy("ALVideoDevice", ip, port)
    subscriberName = videoProxy.subscribe("python_client", 0, 11, 5)
    naoImage = videoProxy.getImageRemote(subscriberName)
    videoProxy.unsubscribe(subscriberName)

    width = naoImage[0]
    height = naoImage[1]
    imageBuffer = naoImage[6]
    image = np.frombuffer(imageBuffer, dtype=np.uint8).reshape((height, width, 3))
    return image

# Main Execution
if __name__ == "__main__":
    video_thread = threading.Thread(target=video_capture_thread)
    motion_thread = threading.Thread(target=motion_control_thread)
    video_thread.start()
    motion_thread.start()
    video_thread.join()
    motion_thread.join()
    cv2.destroyAllWindows()