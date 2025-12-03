#include "Drivetrain.h"

Drivetrain dt(
    48, 50, 52, // Motor A IN1, IN2, ENA
    49, 51, 53, // Motor B IN3, IN4, ENB
    21, 33,     // Encoder A (A pin interrupt, B pin)
    20, 32      // Encoder B (A pin interrupt, B pin)
);

void setup()
{
        Serial.begin(57600);
        dt.attachInterrupts();
}

void loop()
{
        if (Serial.available())
        {
                String cmd = Serial.readStringUntil('\n');
                cmd.trim();

                if (cmd.startsWith("FWD"))
                {
                        int spd = cmd.substring(4).toInt();
                        dt.forward(spd);
                }
                else if (cmd.startsWith("BACK"))
                {
                        int spd = cmd.substring(5).toInt();
                        dt.backward(spd);
                }
                else if (cmd.startsWith("TURNL"))
                {
                        int spd = cmd.substring(6).toInt();
                        dt.left(spd);
                }
                else if (cmd.startsWith("TURNR"))
                {
                        int spd = cmd.substring(6).toInt();
                        dt.right(spd);
                }
                else if (cmd == "STOP")
                {
                        dt.stop();
                }
                else if (cmd.startsWith("SET_PID"))
                {
                        // Example: SET_PID 20 1.2 0.5
                        float kp, ki, kd;
                        sscanf(cmd.c_str(), "SET_PID %f %f %f", &kp, &ki, &kd);
                        dt.setPID(kp, ki, kd);
                }
        }

        // Optional: Arduino sends encoder RPM updates to Pi
        Serial.print("RPM L:");
        Serial.print(dt.leftRPM);
        Serial.print(" R:");
        Serial.println(dt.rightRPM);
}
