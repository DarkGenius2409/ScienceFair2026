import cv2
from picamera2 import Picamera2
from roboflow import Roboflow
import serial
import time
import math

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
APPROACH_THRESHOLD = 20   # cm
BASE_SPEED = 150
ARM_POSITIONS = {
    "LOW": (500, 500),
    "MID": (800, 800),
    "HIGH": (1200, 1200)
}

# ---------------- SERIAL ----------------
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

# ---------------- HELPER ----------------
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

# ---------------- STATE MACHINE ----------------
STATE_SEARCH = 0
STATE_APPROACH = 1
STATE_ALIGN_ARM = 2
STATE_GRAB = 3
STATE_RETRACT = 4

state = STATE_SEARCH
current_object = None

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

    # State logic
    if state == STATE_SEARCH:
        send_cmd("DRIVE 100 100")  # roam slowly
        if result.get("predictions"):
            current_object = result["predictions"][0]
            state = STATE_APPROACH

    elif state == STATE_APPROACH:
        x_robot, y_robot = pixel_to_robot_frame(current_object["x"], current_object["y"])
        dist = distance(x_robot, y_robot)
        if dist > APPROACH_THRESHOLD:
            left_speed, right_speed = compute_drive_commands(x_robot, y_robot)
            send_cmd(f"DRIVE {left_speed} {right_speed}")
        else:
            send_cmd("DRIVE 0 0")
            state = STATE_ALIGN_ARM

    elif state == STATE_ALIGN_ARM:
        # Use a discrete preset based on distance
        if dist < 10:
            arm_low, arm_high = ARM_POSITIONS["LOW"]
        elif dist < 20:
            arm_low, arm_high = ARM_POSITIONS["MID"]
        else:
            arm_low, arm_high = ARM_POSITIONS["HIGH"]

        send_cmd(f"ARM_POS 1 {arm_low}")
        send_cmd(f"ARM_POS 2 {arm_high}")
        state = STATE_GRAB

    elif state == STATE_GRAB:
        send_cmd("ARM_GRAB CLOSE")
        time.sleep(1)
        state = STATE_RETRACT

    elif state == STATE_RETRACT:
        send_cmd(f"ARM_POS 1 {ARM_POSITIONS['HIGH'][0]}")
        send_cmd(f"ARM_POS 2 {ARM_POSITIONS['HIGH'][1]}")
        time.sleep(1)
        state = STATE_SEARCH
