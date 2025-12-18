import cv2
from picamera2 import Picamera2
from roboflow import Roboflow
import serial
import time
import math

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
PICKUP_THRESHOLD = 20  # cm
BASE_SPEED = 150       # drive speed
L1, L2 = 12.0, 10.0   # arm lengths in cm

ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
time.sleep(2)

def send_cmd(cmd):
    ser.write((cmd + "\n").encode())

# ---------------- CAMERA ----------------
picam2 = Picamera2()
picam2.start()

# ---------------- ROBOFLOW ----------------
rf = Roboflow(api_key="YOUR_KEY")
project = rf.workspace().project("trash-detection")
model = project.version(1).model

# ---------------- HELPER FUNCTIONS ----------------
def inverse_kinematics(x, y, L1, L2):
    r = math.hypot(x, y)
    theta2 = math.acos((r**2 - L1**2 - L2**2) / (2*L1*L2))
    theta1 = math.atan2(y, x) - math.atan2(L2*math.sin(theta2), L1 + L2*math.cos(theta2))
    return theta1, theta2

def angle_to_ticks(theta):
    TICKS_PER_REV = 1440
    GEAR_RATIO = 1
    return int(theta / (2*math.pi) * TICKS_PER_REV * GEAR_RATIO)

def pixel_to_robot_frame(x_px, y_px):
    scale = 0.1
    x = (x_px - 320) * scale
    y = (y_px - 240) * scale
    return x, y

def distance(x, y):
    return math.hypot(x, y)

def compute_drive_commands(x, y):
    Kp = 2.0
    angle_error = math.atan2(y, x)
    left_speed = BASE_SPEED - int(Kp * angle_error * 50)
    right_speed = BASE_SPEED + int(Kp * angle_error * 50)
    return max(min(left_speed, 255), -255), max(min(right_speed, 255), -255)

# ---------------- MAIN LOOP ----------------
while True:
    frame = picam2.capture_array()
    result = model.predict(frame).json()

    # Draw bounding boxes
    for obj in result.get("predictions", []):
        x, y = int(obj["x"]), int(obj["y"])
        w, h = int(obj["width"]), int(obj["height"])
        cv2.rectangle(frame, (x-w//2, y-h//2), (x+w//2, y+h//2), (0,255,0), 2)
        cv2.putText(frame, obj["class"], (x-w//2, y-h//2-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    cv2.imshow("Trash Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Autonomous behavior
    if result.get("predictions"):
        obj = result["predictions"][0]
        x_robot, y_robot = pixel_to_robot_frame(obj["x"], obj["y"])
        dist = distance(x_robot, y_robot)

        # Drive toward trash
        if dist > PICKUP_THRESHOLD:
            left_speed, right_speed = compute_drive_commands(x_robot, y_robot)
            send_cmd(f"DRIVE {left_speed} {right_speed}")
        else:
            send_cmd("DRIVE 0 0")
            # Arm pickup
            theta1, theta2 = inverse_kinematics(x_robot, y_robot, L1, L2)
            ticks1, ticks2 = angle_to_ticks(theta1), angle_to_ticks(theta2)
            send_cmd(f"ARM_POS 1 {ticks1}")
            send_cmd(f"ARM_POS 2 {ticks2}")
            send_cmd("ARM_GRAB CLOSE")
    else:
        # No trash detected
        send_cmd("DRIVE 0 0")

    # Optional: read Arduino status
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            print(line)
