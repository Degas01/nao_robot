import cv2
import numpy as np
import heapq
import random
import math
import os
import urllib2  

# Function to download and load images 
def get_image(url, filename, width, height):
    if not os.path.exists(filename):
        print("Downloading {}...".format(filename))
        with open(filename, 'wb') as f:
            response = urllib2.urlopen(url)
            f.write(response.read())
    
    img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    if img is None:
        # Create a colored rectangle as fallback
        img = np.zeros((height, width, 4), dtype=np.uint8)
        if "nao" in filename.lower():
            cv2.rectangle(img, (0, 0), (width, height), (255, 0, 0, 255), -1)
        else:
            cv2.circle(img, (width//2, height//2), width//2, (0, 255, 255, 255), -1)
    else:
        # Resize image to desired dimensions
        img = cv2.resize(img, (width, height))
        
        # Add alpha channel if not present
        if img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    
    return img

# Download/load images
nao_url = "https://upload.wikimedia.org/wikipedia/commons/thumb/e/e0/NAO_Robot_%28Robocup_2016%29.jpg/320px-NAO_Robot_%28Robocup_2016%29.jpg"
ball_url = "https://upload.wikimedia.org/wikipedia/commons/4/41/Tennis_ball.svg"

nao_img = get_image(nao_url, "nao_robot.jpg", 40, 60)
ball_img = get_image(ball_url, "tennis_ball.png", 30, 30)

# Initialize court
WIDTH, HEIGHT = 800, 600
court = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
court[:] = (50, 150, 50)  # Tennis court green

# Court markings
cv2.rectangle(court, (50, 50), (WIDTH-50, HEIGHT-50), (255, 255, 255), 2)
cv2.line(court, (WIDTH//2, 50), (WIDTH//2, HEIGHT-50), (200, 200, 200), 4)  # Net

# Robot and ball
ball_pos = [700, 300]
nao_pos = [100, 300]
nao_path = []
nao_speed = 3  # pixels per frame
nao_direction = 0  # Angle in radians

# Obstacle parameters
OBSTACLE_RADIUS = 25
ROBOT_RADIUS = 20
SAFETY_MARGIN = 15
MIN_DISTANCE = OBSTACLE_RADIUS + ROBOT_RADIUS + SAFETY_MARGIN

def generate_obstacles():
    """Generate obstacles ensuring path exists"""
    obstacles = []
    for _ in range(8):
        while True:
            x = random.randint(150, WIDTH-150)
            y = random.randint(150, HEIGHT-150)
            if (math.hypot(x - nao_pos[0], y - nao_pos[1]) > 200 and 
                math.hypot(x - ball_pos[0], y - ball_pos[1]) > 200):
                obstacles.append((x, y))
                break
    return obstacles

obstacles = generate_obstacles()

def is_collision(x, y):
    """Collision detection"""
    for (ox, oy) in obstacles:
        if math.hypot(ox - x, oy - y) < MIN_DISTANCE:
            return True
    return False

def get_neighbors(node):
    """8-way movement with adaptive step sizes"""
    x, y = node
    neighbors = []
    step = 15  # Base step size
    for dx, dy in [(step,0), (-step,0), (0,step), (0,-step),
                   (step,step), (step,-step), (-step,step), (-step,-step)]:
        nx, ny = x + dx, y + dy
        if (50 <= nx < WIDTH-50 and 50 <= ny < HEIGHT-50):
            if not is_collision(nx, ny):
                neighbors.append((nx, ny))
    return neighbors

def astar(start, goal):
    """A* algorithm with continuous pathfinding"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: math.hypot(goal[0]-start[0], goal[1]-start[1])}
    open_set_hash = {start}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        open_set_hash.remove(current)
        
        if math.hypot(current[0]-goal[0], current[1]-goal[1]) < 20:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path = path[::-1] or [goal]
            return path
        
        for neighbor in get_neighbors(current):
            dist = math.hypot(neighbor[0]-current[0], neighbor[1]-current[1])
            tentative_g = g_score[current] + dist
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + math.hypot(
                    goal[0]-neighbor[0], goal[1]-neighbor[1])
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    # Fallback: adaptive straight-line path with more granular steps
    direct_path = []
    steps = max(1, int(math.hypot(goal[0]-start[0], goal[1]-start[1])/5))  # More granular
    for i in range(1, steps+1):
        x = start[0] + (goal[0]-start[0])*i/steps
        y = start[1] + (goal[1]-start[1])*i/steps
        if not is_collision(int(x), int(y)):
            direct_path.append((int(x), int(y)))
        else:
            # Stop at first collision - return path up to this point
            break
            
    return direct_path if direct_path else []

# Function to overlay an image with transparency
def overlay_image(background, foreground, x, y, angle=0):
    # Get dimensions
    h, w = foreground.shape[:2]
    
    # Rotate image if needed
    if angle != 0:
        # Calculate rotation matrix
        center = (w//2, h//2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle * 180 / math.pi, 1.0)
        foreground = cv2.warpAffine(foreground, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR)
    
    # Calculate positions
    x_offset = int(x - w//2)
    y_offset = int(y - h//2)
    
    # Check if the image is within frame boundaries
    if x_offset < 0 or y_offset < 0 or x_offset + w > background.shape[1] or y_offset + h > background.shape[0]:
        # Adjust offset and dimensions to fit within the frame
        x_start = max(0, x_offset)
        y_start = max(0, y_offset)
        x_end = min(background.shape[1], x_offset + w)
        y_end = min(background.shape[0], y_offset + h)
        
        # Calculate corresponding region in the foreground
        fg_x_start = max(0, -x_offset)
        fg_y_start = max(0, -y_offset)
        fg_x_end = fg_x_start + (x_end - x_start)
        fg_y_end = fg_y_start + (y_end - y_start)
        
        # Extract regions
        roi = background[y_start:y_end, x_start:x_end]
        fg_roi = foreground[fg_y_start:fg_y_end, fg_x_start:fg_x_end]
        
        # Skip if dimensions don't match
        if roi.shape[:2] != fg_roi.shape[:2]:
            return background
        
        # Apply alpha blending
        if fg_roi.shape[2] == 4:  # With alpha channel
            alpha = fg_roi[:, :, 3] / 255.0
            for c in range(3):
                roi[:, :, c] = roi[:, :, c] * (1 - alpha) + fg_roi[:, :, c] * alpha
    else:
        # Image fits entirely within the frame
        roi = background[y_offset:y_offset+h, x_offset:x_offset+w]
        if foreground.shape[2] == 4:  # With alpha channel
            alpha = foreground[:, :, 3] / 255.0
            for c in range(3):
                roi[:, :, c] = roi[:, :, c] * (1 - alpha) + foreground[:, :, c] * alpha
    
    return background

# Path planning variables
frame_count = 0
path_finding_attempts = 0
max_path_finding_attempts = 5  # Limit attempts per frame
smoothing_factor = 0.2  # For smooth direction changes

# Initialize path
nao_path = astar((int(nao_pos[0]), int(nao_pos[1])), (ball_pos[0], ball_pos[1]))

while True:
    frame_count += 1
    frame = court.copy()
    path_finding_attempts = 0  # Reset counter each frame
    
    # Draw obstacles
    for (x, y) in obstacles:
        cv2.circle(frame, (x, y), OBSTACLE_RADIUS, (0, 0, 255), -1)
        cv2.circle(frame, (x, y), MIN_DISTANCE, (0, 100, 100), 1)
    
    # Movement logic
    moved_this_frame = False
    
    if nao_path:
        # Always move toward next waypoint
        dx = nao_path[0][0] - nao_pos[0]
        dy = nao_path[0][1] - nao_pos[1]
        dist = math.hypot(dx, dy)
        
        # Update robot direction 
        target_angle = math.atan2(dy, dx)
        angle_diff = (target_angle - nao_direction + math.pi) % (2 * math.pi) - math.pi
        nao_direction += angle_diff * smoothing_factor
        
        if dist < 5:  # Close to waypoint
            nao_path.pop(0)
            if not nao_path:  # Reached end of path
                # If the robot reach the ball, it generates a new random target
                ball_pos[0] = random.randint(50, WIDTH-50)
                ball_pos[1] = random.randint(50, HEIGHT-50)
                nao_path = astar((int(nao_pos[0]), int(nao_pos[1])), 
                                (ball_pos[0], ball_pos[1]))
        else:
            # Calculate movement vector
            move_x = (dx/dist) * nao_speed
            move_y = (dy/dist) * nao_speed
            
            new_x = nao_pos[0] + move_x
            new_y = nao_pos[1] + move_y
            
            # Check if next position would cause collision
            if is_collision(int(new_x), int(new_y)):
                # Collision detected: replan from current position
                nao_path = astar((int(nao_pos[0]), int(nao_pos[1])), 
                               (ball_pos[0], ball_pos[1]))
                path_finding_attempts += 1
                
                # If a path still exists, try to move along it
                if nao_path and path_finding_attempts < max_path_finding_attempts:
                    # Attempt smaller step in new direction
                    dx = nao_path[0][0] - nao_pos[0]
                    dy = nao_path[0][1] - nao_pos[1]
                    dist = math.hypot(dx, dy)
                    
                    if dist > 0:
                        # Take a smaller step in the new direction
                        move_x = (dx/dist) * (nao_speed * 0.5)  # Reduced speed when replanning
                        move_y = (dy/dist) * (nao_speed * 0.5)
                        new_x = nao_pos[0] + move_x
                        new_y = nao_pos[1] + move_y
                        
                        if not is_collision(int(new_x), int(new_y)):
                            nao_pos[0] = new_x
                            nao_pos[1] = new_y
                            moved_this_frame = True
            else:
                # No collision, move normally
                nao_pos[0] = new_x
                nao_pos[1] = new_y
                moved_this_frame = True
    
    # If the Nao robot has no path or failed to move, try planning again
    if not nao_path or not moved_this_frame:
        if path_finding_attempts < max_path_finding_attempts:
            # Try to find a new path with a slightly different starting point
            # This helps when stuck against an obstacle
            for offset_x, offset_y in [(0,0), (5,0), (-5,0), (0,5), (0,-5)]:
                start_x = int(nao_pos[0]) + offset_x
                start_y = int(nao_pos[1]) + offset_y
                
                # Skip if starting position would be a collision
                if is_collision(start_x, start_y):
                    continue
                    
                nao_path = astar((start_x, start_y), (ball_pos[0], ball_pos[1]))
                path_finding_attempts += 1
                
                if nao_path:
                    break
    
    # Draw path
    if nao_path:
        for i in range(len(nao_path)-1):
            cv2.line(frame, nao_path[i], nao_path[i+1], (0, 255, 0), 2)
    
    # Draw ball image (tennis ball)
    frame = overlay_image(frame, ball_img, ball_pos[0], ball_pos[1])
    
    # Draw robot image (NAO)
    frame = overlay_image(frame, nao_img, int(nao_pos[0]), int(nao_pos[1]), nao_direction)
    
    # Status display
    status = "Pos: ({:.0f},{:.0f}) | Path: {} | Speed: {:.1f} | Moved: {}".format(
        nao_pos[0], nao_pos[1], len(nao_path), nao_speed, "Yes" if moved_this_frame else "No")
    cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Display and controls
    cv2.imshow("NAO Robot - A* Path Planning Algorithm In Function", frame)
    key = cv2.waitKey(30)
    if key == ord('q'):
        break
    elif key == ord('r'):
        nao_pos = [100, 300]
        nao_path = astar((int(nao_pos[0]), int(nao_pos[1])), 
                        (ball_pos[0], ball_pos[1]))
    elif key == ord('n'):
        obstacles = generate_obstacles()
        nao_path = astar((int(nao_pos[0]), int(nao_pos[1])), 
                        (ball_pos[0], ball_pos[1]))
    elif key == ord('+'):
        nao_speed = min(10, nao_speed + 0.5)
    elif key == ord('-'):
        nao_speed = max(1, nao_speed - 0.5)

cv2.destroyAllWindows()