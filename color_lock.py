import cv2
import numpy as np
import serial
import time

# Initialize tracking color and threshold
track_color = None
color_threshold = 20

# Pan-Tilt Servo serial port settings
serial_port = 'COM1'
baud_rate = 9600

# Servo control commands
pan_center_cmd = 'P90'
tilt_center_cmd = 'T90'
pan_angle = 0
tilt_angle = 0
mask=0

# Initialize serial communication
ser = serial.Serial(serial_port, baud_rate)
time.sleep(2)  # Wait for the serial connection to establish

# Function to move the servos
def move_servos(pan_angle, tilt_angle):
    pan_cmd = f'P{pan_angle}'
    tilt_cmd = f'T{tilt_angle}'
    ser.write(pan_cmd.encode())
    ser.write(tilt_cmd.encode())

# Mouse callback function to select tracking color
def select_color(event, x, y, flags, param):
    global track_color

    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = frame[y, x]
        track_color = (r, g, b)

# Main loop
cap = cv2.VideoCapture(0)
cv2.namedWindow('Particle Tracking')
cv2.setMouseCallback('Particle Tracking', select_color)

while True:
    time.sleep(0.05)
    # Capture frame from USB camera
    ret, frame = cap.read()
    if not ret:
        print('Failed to capture frame from USB camera')
        break

    # Find the particle by color
    target_x, target_y = -1, -1
    if track_color is not None:
        if track_color[0] - color_threshold < 0:
            lr=0
        else:
            lr=track_color[0] - color_threshold
        if track_color[1] - color_threshold < 0:
            lg=0
        else:
            lg=track_color[1] - color_threshold
        if track_color[2] - color_threshold < 0:
            lb=0
        else:
            lb=track_color[2] - color_threshold
        lower_color = np.array([lr, lg, lb],np.uint8)
        if track_color[0] + color_threshold > 255:
            ur=255
        else:
            ur=track_color[0] + color_threshold
        if track_color[1] + color_threshold > 255:
            ug=255
        else:
            ug=track_color[1] + color_threshold
        if track_color[2] + color_threshold > 255:
            ub=255
        else:
            ub=track_color[2] + color_threshold
        upper_color = np.array([ur, ug, ub],np.uint8)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(rgb, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            target_x, target_y = x + w // 2, y + h // 2

    # Move the servos to keep the particle in the center
    if target_x != -1 and target_y != -1:
        pan_angle = np.interp(target_x, [0, frame.shape[1]], [0, 180])
        tilt_angle = np.interp(target_y, [0, frame.shape[0]], [0, 180])
        move_servos(int(pan_angle), int(tilt_angle))
    else:
        # If no particle found, center the servos
        ser.write(pan_center_cmd.encode())
        ser.write(tilt_center_cmd.encode())

    # Draw a square around the tracked particle
    if target_x != -1 and target_y != -1:
        size = 40  # Size of the square
        top_left = (target_x - size // 2, target_y - size // 2)
        bottom_right = (target_x + size // 2, target_y + size // 2)
        #cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
        cv2.rectangle(frame, top_left, bottom_right, (0, 0, 255), 2)
        # Define the starting and ending points of the line
        start_point = (target_x, target_y)
        end_point = (target_x + int(pan_angle) - 90, target_y + int(tilt_angle) - 90)
        # Draw the line on the image
        cv2.line(frame, start_point, end_point, (0, 255, 0), 2)


    # Print pan, tilt, and color at the bottom of the image
    pan_text = f'Pan: {int(pan_angle)}'
    tilt_text = f'Tilt: {int(tilt_angle)}'
    color_text = f'Color: {track_color}'
    cv2.putText(frame, pan_text, (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, tilt_text, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, color_text, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    # Display the frame with particle tracking
    cv2.imshow('Particle Tracking', frame)
    cv2.imshow("Detect", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
ser.close()
