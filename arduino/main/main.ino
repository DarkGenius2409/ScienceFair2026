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
        dt.setSpeed(120, 120); // command both wheels to 120 RPM
        dt.updatePID();
}
