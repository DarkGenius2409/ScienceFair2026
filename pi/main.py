import cv2
from picamera2 import Picamera2
from roboflow import Roboflow
import serial
import time
import threading

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
BASE_SPEED = 150
ARM_SPEED = 100

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

# ---------------- SERIAL READER ----------------
def read_serial():
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print("Arduino:", line)

# Start serial reading in a separate thread
threading.Thread(target=read_serial, daemon=True).start()

# ---------------- TELEOP ----------------
def handle_keys(key):
    # Drive control
    if key == ord('w'):
        send_cmd(f"DRIVE {BASE_SPEED} {BASE_SPEED}")
    elif key == ord('s'):
        send_cmd(f"DRIVE {-BASE_SPEED} {-BASE_SPEED}")
    elif key == ord('a'):
        send_cmd(f"DRIVE {-BASE_SPEED} {BASE_SPEED}")
    elif key == ord('d'):
        send_cmd(f"DRIVE {BASE_SPEED} {-BASE_SPEED}")
    elif key == ord('x'):
        send_cmd("DRIVE 0 0")  # Stop

    # Arm control
    elif key == ord('i'):
        send_cmd(f"MANUAL ARM1 {ARM_SPEED}")
    elif key == ord('k'):
        send_cmd(f"MANUAL ARM1 {-ARM_SPEED}")
    elif key == ord('j'):
        send_cmd(f"MANUAL ARM2 {ARM_SPEED}")
    elif key == ord('l'):
        send_cmd(f"MANUAL ARM2 {-ARM_SPEED}")

    # Claw control
    elif key == ord('o'):
        send_cmd("ARM_GRAB OPEN")
    elif key == ord('p'):
        send_cmd("ARM_GRAB CLOSE")

    # Request encoder counts
    elif key == ord('e'):
        send_cmd("STATUS")

# ---------------- MAIN LOOP ----------------
print("Teleop controls:")
print("W/A/S/D: drive, X: stop")
print("I/K: arm1, J/L: arm2")
print("O/P: claw open/close, E: encoder status")
print("Press 'q' to quit\n")

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

    cv2.imshow("Teleop Trash Robot", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key != 255:
        handle_keys(key)

cv2.destroyAllWindows()
ser.close()
