#include "Drivetrain.h"

DrivetrainPID *Drivetrain::instance = nullptr;

// -----------------------------------------
// CONSTRUCTOR
// -----------------------------------------
Drivetrain::Drivetrain(
    int in1, int in2, int ena,
    int in3, int in4, int enb,
    int encA_A, int encA_B,
    int encB_A, int encB_B)
{
        this->in1 = in1;
        this->in2 = in2;
        this->ena = ena;
        this->in3 = in3;
        this->in4 = in4;
        this->enb = enb;

        this->encA_A = encA_A;
        this->encA_B = encA_B;
        this->encB_A = encB_A;
        this->encB_B = encB_B;

        instance = this;

        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(ena, OUTPUT);
        pinMode(in3, OUTPUT);
        pinMode(in4, OUTPUT);
        pinMode(enb, OUTPUT);

        pinMode(encA_A, INPUT);
        pinMode(encA_B, INPUT);
        pinMode(encB_A, INPUT);
        pinMode(encB_B, INPUT);

        lastAState = digitalRead(encA_A);
        lastBState = digitalRead(encB_A);
}

// -----------------------------------------
// INTERRUPTS
// -----------------------------------------
void DrivetrainPID::attachInterrupts()
{
        attachInterrupt(digitalPinToInterrupt(encA_A), encoderA_ISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encB_A), encoderB_ISR, CHANGE);
}

void Drivetrain::encoderA_ISR()
{
        Drivetrain *dt = instance;

        int state = digitalRead(dt->encA_A);
        if (dt->lastAState == LOW && state == HIGH)
        {
                int val = digitalRead(dt->encA_B);
                dt->dirA = (val == HIGH);
        }
        dt->lastAState = state;

        if (dt->dirA)
                dt->pulsesA++;
        else
                dt->pulsesA--;
}

void Drivetrain::encoderB_ISR()
{
        Drivetrain *dt = instance;

        int state = digitalRead(dt->encB_A);
        if (dt->lastBState == LOW && state == HIGH)
        {
                int val = digitalRead(dt->encB_B);
                dt->dirB = (val == HIGH);
        }
        dt->lastBState = state;

        if (dt->dirB)
                dt->pulsesB++;
        else
                dt->pulsesB--;
}

// -----------------------------------------
// MOTOR LOW-LEVEL
// -----------------------------------------
void Drivetrain::setMotor(int inA, int inB, int pwmPin, int speed)
{
        speed = constrain(speed, -255, 255);

        if (speed >= 0)
        {
                digitalWrite(inA, HIGH);
                digitalWrite(inB, LOW);
                analogWrite(pwmPin, speed);
        }
        else
        {
                digitalWrite(inA, LOW);
                digitalWrite(inB, HIGH);
                analogWrite(pwmPin, -speed);
        }
}

// -----------------------------------------
// RPM UPDATE
// -----------------------------------------
void Drivetrain::updateRPM()
{
        if (millis() - lastRPMTime >= 100)
        {
                rpmA = (pulsesA * 600.0) / PPR;
                rpmB = (pulsesB * 600.0) / PPR;

                pulsesA = pulsesB = 0;
                lastRPMTime = millis();
        }
}

// -----------------------------------------
// PID LOOP
// -----------------------------------------
void Drivetrain::updatePID()
{
        updateRPM();

        float dt = 0.1;

        // LEFT MOTOR PID
        float errA = targetRPM_A - rpmA;
        integralA += errA * dt;
        float derivA = (errA - prevErrA) / dt;
        prevErrA = errA;

        pwmA += kP * errA + kI * integralA + kD * derivA;
        pwmA = constrain(pwmA, -255, 255);

        // RIGHT MOTOR PID
        float errB = targetRPM_B - rpmB;
        integralB += errB * dt;
        float derivB = (errB - prevErrB) / dt;
        prevErrB = errB;

        pwmB += kP * errB + kI * integralB + kD * derivB;
        pwmB = constrain(pwmB, -255, 255);

        // APPLY PWM
        setMotor(in1, in2, ena, pwmA);
        setMotor(in3, in4, enb, pwmB);
}

// -----------------------------------------
// PRESERVED ORIGINAL METHODS
// -----------------------------------------
void Drivetrain::drive(int leftPWM, int rightPWM)
{
        setMotor(in1, in2, ena, leftPWM);
        setMotor(in3, in4, enb, rightPWM);
}

void Drivetrain::forward(int speedPWM)
{
        drive(speedPWM, speedPWM);
}

void Drivetrain::backward(int speedPWM)
{
        drive(-speedPWM, -speedPWM);
}

void Drivetrain::left(int speedPWM)
{
        drive(speedPWM * 0.5, speedPWM);
}

void Drivetrain::right(int speedPWM)
{
        drive(speedPWM, speedPWM * 0.5);
}

void Drivetrain::pivotLeft(int speedPWM)
{
        drive(-speedPWM, speedPWM);
}

void Drivetrain::pivotRight(int speedPWM)
{
        drive(speedPWM, -speedPWM);
}

void Drivetrain::stop()
{
        drive(0, 0);
}

// -----------------------------------------
// NEW CLOSED-LOOP CONTROL
// -----------------------------------------
void Drivetrain::setSpeedRPM(float leftRPM, float rightRPM)
{
        targetRPM_A = leftRPM;
        targetRPM_B = rightRPM;
}
