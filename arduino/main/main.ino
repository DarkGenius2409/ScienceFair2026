#include "../Drivetrain.h"

Drivetrain dt(
    8, 9, 5,   // Motor A IN1, IN2, ENA
    10, 11, 6, // Motor B IN3, IN4, ENB
    2, 4,      // Encoder A (A pin interrupt, B pin)
    3, 7       // Encoder B (A pin interrupt, B pin)
);

void setup()
{
        Serial.begin(57600);
        dt.attachInterrupts();
}

void loop()
{
        dt.setSpeed(120, 120); // command both wheels to 120 RPM
        dt.updatePID();
}
