# -*- coding: utf-8 -*-

import cv2
import numpy as np

# Uncomment these lines if running on NAO and ensure NAOqi SDK is installed
from naoqi import ALProxy

def get_frame_from_nao(ip, port):
    """
    Connects to NAO's ALVideoDevice, retrieves one frame,
    and converts it to an OpenCV image.
    """
    videoProxy = ALProxy("ALVideoDevice", ip, port)
    # Subscribe to the video feed (camera index 0, resolution kQVGA, color space kBGR)
    # Note: Adjust parameters if needed; here 11 denotes kBGR color space.
    subscriberName = videoProxy.subscribeCamera("python_client", 0, 1, 11, 5)

    naoImage = videoProxy.getImageRemote(subscriberName)
    videoProxy.unsubscribe(subscriberName)
    
    # Extract image info (naoImage[0]: width, [1]: height, [6]: image buffer)
    width = naoImage[0]
    height = naoImage[1]
    imageBuffer = naoImage[6]
    
    # Convert the raw image buffer to a numpy array and reshape to height x width x 3 (BGR)
    image = np.frombuffer(imageBuffer, dtype=np.uint8).reshape((height, width, 3))
    return image

def detect_ball(frame):
    """
    Detects a tennis ball in the given frame using HSV color thresholding and contour detection.
    Returns the frame annotated with the detected ball (if any) and the ball's coordinates.
    """
    # Convert frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV range for tennis ball color (adjust these values for your lighting)
    lower_yellow = np.array([75, 60, 90])
    upper_yellow = np.array([100, 180, 210])
    
    # Create mask where the color falls within the range
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Optional: clean up the mask using erosion and dilation
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ball_info = None
    
    if contours:
        # Assume the largest contour is the ball
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 10:  # adjust threshold to ignore noise
            # Draw the circle around the detected ball
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.putText(frame, "Ball", (int(x - radius), int(y - radius)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            ball_info = {"x": int(x), "y": int(y), "radius": int(radius)}
    return frame, ball_info

def main():
    # Choose whether to use NAO's camera feed or a local webcam for testing
    use_nao = True  # Set to True if running on NAO V5
    if use_nao:
        ip = "192.168.0.108"  # NAO robot's IP address
        port = 9559
    else:
        cap = cv2.VideoCapture(0)
    
    while True:
        # Capture frame from NAO or local webcam
        if use_nao:
            frame = get_frame_from_nao(ip, port)
        else:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
        
        # Detect ball in the frame
        annotated_frame, ball = detect_ball(frame)
        if ball:
            print("Ball detected at (x={}, y={}) with radius {}".format(ball["x"], ball["y"], ball["radius"]))
        
        # Display the annotated frame
        cv2.imshow("Ball Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    if not use_nao:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
