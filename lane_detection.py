#!/usr/bin/env python3
# Lane detection with GPIO control for Raspberry Pi 4
# Compatible with OpenCV 4.x

import cv2
import numpy as np
import math
import time
import RPi.GPIO as GPIO

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor control pins
m11 = 16
m12 = 12
m21 = 21
m22 = 20

# Set up GPIO pins
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

# Set all motors off initially
GPIO.output(m11, GPIO.LOW)
GPIO.output(m12, GPIO.LOW)
GPIO.output(m21, GPIO.LOW)
GPIO.output(m22, GPIO.LOW)

def move_forward():
    """Move the robot forward"""
    GPIO.output(m11, GPIO.HIGH)
    GPIO.output(m12, GPIO.LOW)
    GPIO.output(m21, GPIO.HIGH)
    GPIO.output(m22, GPIO.LOW)
    print("Moving FORWARD")

def turn_left():
    """Turn the robot left"""
    GPIO.output(m11, GPIO.LOW)
    GPIO.output(m12, GPIO.HIGH)
    GPIO.output(m21, GPIO.HIGH)
    GPIO.output(m22, GPIO.LOW)
    print("Turning LEFT")

def turn_right():
    """Turn the robot right"""
    GPIO.output(m11, GPIO.HIGH)
    GPIO.output(m12, GPIO.LOW)
    GPIO.output(m21, GPIO.LOW)
    GPIO.output(m22, GPIO.HIGH)
    print("Turning RIGHT")

def stop():
    """Stop the robot"""
    GPIO.output(m11, GPIO.LOW)
    GPIO.output(m12, GPIO.LOW)
    GPIO.output(m21, GPIO.LOW)
    GPIO.output(m22, GPIO.LOW)
    print("STOPPED")
    
    
def detect_lanes(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Define region of interest (ROI)
    height, width = gray.shape
    roi_vertices = np.array([
        [(0, height), (width//2 - 50, height//2 + 50), 
         (width//2 + 50, height//2 + 50), (width, height)]
    ], dtype=np.int32)
    
    # Apply ROI mask
    mask = np.zeros_like(gray)
    cv2.fillPoly(mask, roi_vertices, 255)
    masked = cv2.bitwise_and(gray, mask)
    
    # Apply Gaussian blur
    blur = cv2.GaussianBlur(masked, (5, 5), 0)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blur, 50, 150)
    
    # Apply Hough Transform to detect lines
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi/180,
        threshold=30,
        minLineLength=30,
        maxLineGap=100
    )
    
    return edges, lines, roi_vertices

def process_lines(frame, lines):
    if lines is None:
        return frame, "unknown", 0, 0
    
    # Create separate lists for left and right lanes
    left_lines = []
    right_lines = []
    
    # Frame center (horizontal)
    frame_center = frame.shape[1] // 2
    
    # For visualizing the lines on the frame
    line_image = np.zeros_like(frame)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            # Calculate slope
            if x2 - x1 == 0:  # Vertical line
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter lines based on slope
            if abs(slope) < 0.3:  # Horizontal lines, ignore
                continue
                
            # Left lane (negative slope)
            if slope < 0 and x1 < frame_center and x2 < frame_center:
                left_lines.append(line)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            # Right lane (positive slope)
            elif slope > 0 and x1 > frame_center and x2 > frame_center:
                right_lines.append(line)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
    
    # Combine the line image with the original frame
    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
    
    # Determine direction based on lines detected
    left_count = len(left_lines)
    right_count = len(right_lines)
    
    if left_count > 3 and right_count > 3:
        direction = "straight"
    elif left_count > right_count * 2:
        direction = "right"  # More left lines visible when turning right
    elif right_count > left_count * 2:
        direction = "left"   # More right lines visible when turning left
    else:
        direction = "straight"
    
    return combo_image, direction, left_count, right_count


def draw_direction_text(frame, direction):
    # Add text to show the detected direction
    text_position = (20, 50)
    if direction == "left":
        cv2.putText(frame, "LEFT TURN", text_position, 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    elif direction == "right":
        cv2.putText(frame, "RIGHT TURN", text_position, 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "STRAIGHT", text_position, 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return frame

def draw_roi(frame, vertices):
    # Draw the ROI on the frame
    cv2.polylines(frame, [vertices], True, (255, 0, 0), 2)
    return frame

def control_motors(direction):
    """Control motors based on lane detection"""
    if direction == "left":
        turn_left()
    elif direction == "right":
        turn_right()
    elif direction == "straight":
        move_forward()
    else:
        stop()  # Stop if direction is unknown

def lane_detection():
    # Initialize camera
    # Use 0 for default camera, or specify device path like '/dev/video0'
    cap = cv2.VideoCapture(0)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Direction stability counter
    current_direction = "unknown"
    direction_counter = 0
    stable_direction = "unknown"
    STABILITY_THRESHOLD = 5  # Need this many consecutive same readings
    
    print("Lane detection started. Press 'q' to quit.")
    
    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Resize for performance
            frame = cv2.resize(frame, (640, 480))
            
            # Detect lanes
            edges, lines, roi = detect_lanes(frame)
            
            # Process detected lines
            result_frame, direction, left_count, right_count = process_lines(frame.copy(), lines)
            
            # Apply direction stability logic
            if direction == current_direction:
                direction_counter += 1
            else:
                current_direction = direction
                direction_counter = 1
                
            # Only change motor direction when direction is stable
            if direction_counter >= STABILITY_THRESHOLD:
                if stable_direction != current_direction:
                    stable_direction = current_direction
                    control_motors(stable_direction)
            
            # Draw the ROI
            result_frame = draw_roi(result_frame, roi)
            
            # Add direction text
            result_frame = draw_direction_text(result_frame, stable_direction)
            
            # Display status info
            cv2.putText(result_frame, f"Left lines: {left_count}", (20, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(result_frame, f"Right lines: {right_count}", (20, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Display the resulting frames
            cv2.imshow("Edge Detection", edges)
            cv2.imshow("Lane Detection", result_frame)
            
            # Exit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Lane detection stopped by user")
    finally:
        # Clean up
        stop()  # Stop motors
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        print("Lane detection resources released")

if __name__ == "__main__":
    lane_detection()
