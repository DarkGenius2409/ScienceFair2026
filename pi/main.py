import serial
import time

# Change to the Arduino serial port (check with: ls /dev/ttyACM*)
ARDUINO_PORT = "/dev/ttyACM0"
BAUD = 115200

class ArduinoInterface:
    def __init__(self, port=ARDUINO_PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connect()

    def connect(self):
        """Attempt to connect until successful."""
        while True:
            try:
                print(f"Connecting to Arduino on {self.port}...")
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2)  # Wait for Arduino reset
                print("Connected.")
                break
            except Exception as e:
                print("Failed. Retrying...")
                time.sleep(1)

    def send(self, msg):
        """Send a string command to Arduino."""
        if not msg.endswith("\n"):
            msg += "\n"
        self.ser.write(msg.encode())
        self.ser.flush()

    def read_line(self):
        """Read a line if Arduino sends any output."""
        try:
            line = self.ser.readline().decode().strip()
            return line
        except:
            return ""

    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial connection closed.")


def main():
    arduino = ArduinoInterface()

    try:
        while True:

            # Example: send command
            arduino.send("FWD 12")
            print("Sent: FWD 12")

            # Listen for Arduino output (PID debug, encoder RPM, etc.)
            line = arduino.read_line()
            if line:
                print("Arduino:", line)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
        arduino.close()


if __name__ == "__main__":
    main()
