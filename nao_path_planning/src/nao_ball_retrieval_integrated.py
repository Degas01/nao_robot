# -*- coding: utf-8 -*-
import sys
# Add path to NAOqi Python SDK (Ensure this path is correct for your system)
sys.path.append("D:/KCL/TA/7CCEMSAP/NAO/pynaoqi-python2.7-2.8.7.4-win64-vs2015-20210818_210634/pynaoqi-python2.7-2.8.7.4-win64-vs2015-20210818_210634/lib")

import cv2
import numpy as np
from naoqi import ALProxy
import time

# Set the IP address and port for communication with the NAO robot
IP = "169.254.157.168" 
PORT = 9559

# Define tracking method (1: Move towards ball, 2: Track with head only)
TRACKING_METHOD = 1    

def start_proxies():
    """
    Initialize proxies for motion, posture, and memory.
    Wake up the robot and set it to the initial standing posture.
    Adjust head pitch to start looking for the ball.
    """
    motion_proxy = ALProxy("ALMotion", IP, PORT)
    posture_proxy = ALProxy("ALRobotPosture", IP, PORT)
    memory_proxy = ALProxy("ALMemory", IP, PORT)

    motion_proxy.wakeUp() # Wake up the robot
    posture_proxy.goToPosture("StandInit", 0.5) # Set the initial standing posture
    motion_proxy.setAngles("HeadPitch", 0.35, 0.1) # Tilt head slightly down to look for the ball

    return motion_proxy, posture_proxy, memory_proxy

def get_image(video_device, capture_device):
    """
    Capture an image from NAO's camera.
    Convert the raw image data into a format suitable for OpenCV.
    """
    img = video_device.getImageRemote(capture_device)
    width = img[0]
    height = img[1]
    array = np.fromstring(img[6], dtype=np.uint8)
    image = array.reshape((height, width, 3)) # Reshape into an image
    return image

def detect_ball(img):
    """
    Detect a yellow ball in the given image.
    Convert image to HSV and apply color thresholding.
    Find contours and return the largest detected ball position and radius.
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([75, 60, 90]) # Lower HSV bound for yellow
    upper_yellow = np.array([100, 180, 210]) # Upper HSV bound for yellow
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply morphological operations to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours of detected objects
    result = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = result[0] if len(result) == 2 else result[1]  

    if contours:
        # Get the largest detected contour
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        # Only consider objects that are large enough
        if radius > 10:
            
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2) # Draw detected ball
            
            cv2.putText(img, "Ball", (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2) # Label the ball
            return int(x), int(y), int(radius)
    return None, None, None # No ball detected

def move_to_ball(motion_proxy, x, y, width, height):
    """
    Move the robot towards the detected ball.
    Adjusts speed based on the horizontal offset of the ball.
    """
    x_offset = x - width / 2 # Horizontal offset from image center
    normalized_offset = x_offset / (width / 2.0) # Normalize the offset

    forward_speed = 0.5 # Move forward speed
    rotate_speed = -0.5 * normalized_offset # Rotate proportionally to the offset
    
    motion_proxy.moveToward(forward_speed, 0.0, rotate_speed)

def track_with_head(motion_proxy, x, y, width, height):
    """
    Adjust NAO's head to track the ball's position.
    """
    x_offset = x - width / 2 # Horizontal offset from center
    yaw = -x_offset / (width / 2.0) * 0.5  # Compute yaw rotation
    pitch = 0.35 # Keep head slightly down

    motion_proxy.setAngles(["HeadYaw", "HeadPitch"], [yaw, pitch], 0.2)

def kick(motion_proxy, side=0):
    """
    Perform a simple kicking motion.
    side = 0 (left leg), side = 1 (right leg)
    """
    leg = "R" if side == 1 else "L"
    names = [leg + "HipPitch", leg + "KneePitch", leg + "AnklePitch"]
    angles = [-0.4, 0.95, -0.55] # Move leg backward before kicking
    speeds = [0.2, 0.2, 0.2] # Set movement speed

    motion_proxy.setAngles(names, angles, speeds)
    time.sleep(0.5) # Wait before returning to neutral position
    motion_proxy.setAngles(names, [0.0, 0.0, 0.0], speeds) # Reset leg position

def main():
    """
    Main loop for ball tracking and retrieval.
    The robot searches for a ball, moves towards it, and kicks it when close enough.
    """
    motion_proxy, posture_proxy, memory_proxy = start_proxies()
    video_device = ALProxy("ALVideoDevice", IP, PORT)
    capture_device = video_device.subscribeCamera("python_client", 0, 2, 11, 10)

    times_no_ball = 0

    try:
        while True:
            # Capture a frame from the camera
            frame = get_image(video_device, capture_device)
            x, y, r = detect_ball(frame) # Detect ball in the frame

            if x is not None:
                # Track ball with head
                track_with_head(motion_proxy, x, y, frame.shape[1], frame.shape[0])

                if TRACKING_METHOD == 1:
                    # Move towards the ball
                    move_to_ball(motion_proxy, x, y, frame.shape[1], frame.shape[0])
                    
                    # Check if the ball is close enough to kick
                    if y > frame.shape[0] - 50 and abs(x - 0.5 * frame.shape[1]) < 25:
                        kick(motion_proxy, int(x > 0.5 * frame.shape[1]))
                
                times_no_ball = 0 # Reset counter if ball is found

            else:
                times_no_ball += 1
                if times_no_ball > 3:
                    motion_proxy.stopMove() # Stop movement if ball is lost for a while

            # Show the processed frame
            cv2.imshow("Ball Tracker", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Cleanup and shutdown robot safely
        cv2.destroyAllWindows()
        video_device.unsubscribe(capture_device)
        motion_proxy.stopMove()
        motion_proxy.rest()

# Run the main function
if __name__ == "__main__":
    main()
