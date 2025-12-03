#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

class Drivetrain
{
private:
        // Motor pins
        int in1, in2, ena;
        int in3, in4, enb;

        // Encoder pins
        int encA_A, encA_B;
        int encB_A, encB_B;

        // Encoder counters
        volatile long pulsesA = 0;
        volatile long pulsesB = 0;

        volatile boolean dirA = true;
        volatile boolean dirB = true;

        volatile byte lastAState;
        volatile byte lastBState;

        // RPM calculation
        const int PPR = 20; // EDIT FOR YOUR ENCODER
        float rpmA = 0;
        float rpmB = 0;
        unsigned long lastRPMTime = 0;

        // PID variables
        float targetRPM_A = 0;
        float targetRPM_B = 0;

        float kP = 1.3;
        float kI = 0.45;
        float kD = 0.12;

        float prevErrA = 0, prevErrB = 0;
        float integralA = 0, integralB = 0;

        int pwmA = 0, pwmB = 0;

        // Low-level motor driver output
        void setMotor(int inA, int inB, int pwmPin, int speed);

public:
        // Constructor
        Drivetrain(
            int in1, int in2, int ena,
            int in3, int in4, int enb,
            int encA_A, int encA_B,
            int encB_A, int encB_B);

        // ISR hooks
        static DrivetrainPID *instance;
        void attachInterrupts();

        static void encoderA_ISR();
        static void encoderB_ISR();

        // RPM + PID update loops
        void updateRPM();
        void updatePID();

        // ---- Preserved Methods ----
        void drive(int leftPWM, int rightPWM); // raw PWM
        void forward(int speedPWM);
        void backward(int speedPWM);
        void left(int speedPWM);
        void right(int speedPWM);
        void pivotLeft(int speedPWM);
        void pivotRight(int speedPWM);
        void stop();

        // ---- New closed-loop control ----
        void setSpeedRPM(float leftRPM, float rightRPM);
};

#endif
