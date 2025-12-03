#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

class Drivetrain
{
private:
        // --- Motor pins ---
        int in1, in2, ena;
        int in3, in4, enb;

        // --- Encoder pins ---
        int encA_A, encA_B;
        int encB_A, encB_B;

        // Interrupt pulse counters
        volatile long pulsesA = 0;
        volatile long pulsesB = 0;

        volatile boolean dirA = true;
        volatile boolean dirB = true;

        // Last states
        volatile byte lastAstate;
        volatile byte lastBstate;

        // --- RPM calculation ---
        const int PPR = 20; // pulses per revolution (change for your encoder)
        unsigned long lastRPMTime = 0;
        float rpmA = 0;
        float rpmB = 0;

        // --- PID variables ---
        float targetRPM_A = 0;
        float targetRPM_B = 0;

        float kP = 1.2;
        float kI = 0.4;
        float kD = 0.1;

        float errorA_prev = 0, errorB_prev = 0;
        float integralA = 0, integralB = 0;

        int pwmA = 0, pwmB = 0;

        // === Internal helpers ===
        void setMotor(int inA, int inB, int pwmPin, int speed);

public:
        Drivetrain(
            int in1, int in2, int ena,
            int in3, int in4, int enb,
            int encA_A, int encA_B,
            int encB_A, int encB_B);

        // Encoder ISR functions (must be public static)
        static void handleA_ISR();
        static void handleB_ISR();

        // attach ISRs after static linking
        void attachInterrupts();

        // RPM update
        void updateRPM();

        // PID update loop
        void updatePID();

        // Set speed in RPM (closed-loop)
        void setSpeed(float leftRPM, float rightRPM);

        // Stop motors
        void stop();

        // Static accessors for ISR storage (singleton pattern)
        static Drivetrain *instance;
};

#endif
