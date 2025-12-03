#include "Drivetrain.h"

Drivetrain *Drivetrain::instance = nullptr;

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

        lastAstate = digitalRead(encA_A);
        lastBstate = digitalRead(encB_A);
}

void Drivetrain::attachInterrupts()
{
        attachInterrupt(digitalPinToInterrupt(encA_A), handleA_ISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encB_A), handleB_ISR, CHANGE);
}

// ============================================
// ISR: Encoder A
// ============================================
void Drivetrain::handleA_ISR()
{
        Drivetrain *dt = instance;
        int stateA = digitalRead(dt->encA_A);

        if (dt->lastAstate == LOW && stateA == HIGH)
        {
                int val = digitalRead(dt->encA_B);
                dt->dirA = (val == HIGH);
        }

        dt->lastAstate = stateA;

        if (dt->dirA)
                dt->pulsesA++;
        else
                dt->pulsesA--;
}

// ============================================
// ISR: Encoder B
// ============================================
void Drivetrain::handleB_ISR()
{
        Drivetrain *dt = instance;
        int stateB = digitalRead(dt->encB_A);

        if (dt->lastBstate == LOW && stateB == HIGH)
        {
                int val = digitalRead(dt->encB_B);
                dt->dirB = (val == HIGH);
        }

        dt->lastBstate = stateB;

        if (dt->dirB)
                dt->pulsesB++;
        else
                dt->pulsesB--;
}

// ============================================
// MOTOR CONTROL
// ============================================
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

// ============================================
// RPM CALCULATION
// ============================================
void Drivetrain::updateRPM()
{
        if (millis() - lastRPMTime >= 100)
        {                                       // update every 0.1 sec
                rpmA = (pulsesA * 600.0) / PPR; // 600 = 60 sec / 0.1 sec
                rpmB = (pulsesB * 600.0) / PPR;

                pulsesA = 0;
                pulsesB = 0;

                lastRPMTime = millis();
        }
}

// ============================================
// PID UPDATE
// ============================================
void Drivetrain::updatePID()
{
        updateRPM();

        // ---- Left motor PID ----
        float errorA = targetRPM_A - rpmA;
        integralA += errorA * 0.1;
        float derivativeA = (errorA - errorA_prev) / 0.1;
        errorA_prev = errorA;

        pwmA += kP * errorA + kI * integralA + kD * derivativeA;
        pwmA = constrain(pwmA, -255, 255);

        // ---- Right motor PID ----
        float errorB = targetRPM_B - rpmB;
        integralB += errorB * 0.1;
        float derivativeB = (errorB - errorB_prev) / 0.1;
        errorB_prev = errorB;

        pwmB += kP * errorB + kI * integralB + kD * derivativeB;
        pwmB = constrain(pwmB, -255, 255);

        // Apply motor output
        setMotor(in1, in2, ena, pwmA);
        setMotor(in3, in4, enb, pwmB);
}

// ============================================
// Set speed in RPM
// ============================================
void Drivetrain::setSpeed(float leftRPM, float rightRPM)
{
        targetRPM_A = leftRPM;
        targetRPM_B = rightRPM;
}

// ============================================
void Drivetrain::stop()
{
        targetRPM_A = targetRPM_B = 0;
        setMotor(in1, in2, ena, 0);
        setMotor(in3, in4, enb, 0);
}
