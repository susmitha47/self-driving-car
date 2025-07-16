import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Motor pins as per first code
m11 = 16
m12 = 12
m21 = 21
m22 = 20

# GPIO Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

# Movement functions using m11, m12, m21, m22
def forward():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print('Forward')

def backward():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 1)
    print('Backward')

def stop():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    print('Stop')

def right():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    print('Right')

def left():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print('Left')
    

class TrafficSignDetector:
    def __init__(self):
        # Define traffic sign color ranges (HSV)
        # Stop sign (red)
        self.stop_sign_lower = np.array([0, 100, 100], np.uint8)
        self.stop_sign_upper = np.array([10, 255, 255], np.uint8)
        
        # Second range for red (wraps around in HSV)
        self.stop_sign_lower2 = np.array([160, 100, 100], np.uint8)
        self.stop_sign_upper2 = np.array([179, 255, 255], np.uint8)
        
        # Go sign (green)
        self.go_sign_lower = np.array([40, 50, 50], np.uint8)
        self.go_sign_upper = np.array([90, 255, 255], np.uint8)
        
        # Yield sign (yellow)
        self.yield_sign_lower = np.array([20, 100, 100], np.uint8)
        self.yield_sign_upper = np.array([30, 255, 255], np.uint8)
        
        # Minimum area for detection
        self.min_area = 500
        
        # Kernel for morphological operations
        self.kernel = np.ones((5, 5), "uint8")
        
        # Last detected sign and timestamp
        self.last_sign = None
        self.last_detection_time = 0
        self.detection_cooldown = 2  # seconds
    def detect_signs(self, frame):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for different sign colors
        stop_mask1 = cv2.inRange(hsv, self.stop_sign_lower, self.stop_sign_upper)
        stop_mask2 = cv2.inRange(hsv, self.stop_sign_lower2, self.stop_sign_upper2)
        stop_mask = cv2.bitwise_or(stop_mask1, stop_mask2)
        
        go_mask = cv2.inRange(hsv, self.go_sign_lower, self.go_sign_upper)
        yield_mask = cv2.inRange(hsv, self.yield_sign_lower, self.yield_sign_upper)
        
        # Apply morphological operations
        stop_mask = cv2.dilate(stop_mask, self.kernel)
        go_mask = cv2.dilate(go_mask, self.kernel)
        yield_mask = cv2.dilate(yield_mask, self.kernel)
        
        # Find contours for each mask
        stop_contours, _ = cv2.findContours(stop_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        go_contours, _ = cv2.findContours(go_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yield_contours, _ = cv2.findContours(yield_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process results
        result_frame = frame.copy()
        current_time = time.time()
        detected_sign = None
        
        # Check for stop sign (red)
        for contour in stop_contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check aspect ratio for stop sign (close to 1:1)
                aspect_ratio = float(w) / h
                if 0.8 <= aspect_ratio <= 1.2:
                    cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(result_frame, "STOP SIGN", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    detected_sign = "stop"
        
        # Check for go sign (green)
        for contour in go_contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check aspect ratio for circular signs
                aspect_ratio = float(w) / h
                if 0.8 <= aspect_ratio <= 1.2:
                    cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(result_frame, "GO SIGN", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    detected_sign = "go"
        
        # Check for yield sign (yellow)
        for contour in yield_contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check for triangular shape
                approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                if len(approx) == 3:  # Triangle
                    cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(result_frame, "YIELD SIGN", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    detected_sign = "yield"
        
        # Update sign detection if cooldown has passed
        if detected_sign and (current_time - self.last_detection_time) > self.detection_cooldown:
            self.last_sign = detected_sign
            self.last_detection_time = current_time
            
        return result_frame, self.last_sign

# Main function
def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Initialize detector
    detector = TrafficSignDetector()
    
    # Start with default behavior
    forward()
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
                
            # Detect traffic signs
            result_frame, detected_sign = detector.detect_signs(frame)
            
            # Control the robot based on detected sign
            if detected_sign == "stop":
                stop()
                time.sleep(2)  # Stop for 2 seconds
            elif detected_sign == "go":
                forward()
            elif detected_sign == "yield":
                # Slow down (could implement PWM for better control)
                stop()
                time.sleep(0.5)
                forward()
            
            # Display results
            cv2.imshow("Traffic Sign Detection", result_frame)
            
            # Break loop with 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Clean up
        stop()
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

