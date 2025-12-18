// ================= INDEX =================
#define DRIVE_L 0
#define DRIVE_R 1
#define ARM_1 2
#define ARM_2 3
#define CLAW 4

// ================= ENCODERS =================
const byte encA[4] = {2, 3, 21, 20}; // INT0-INT3
const byte encB[4] = {22, 23, 24, 25};

volatile long encCount[4] = {0, 0, 0, 0};
long targetCount[4] = {0, 0, 0, 0};
boolean encDir[4] = {true, true, true, true};

// ================= MOTORS (L298N) =================
const int EN[4] = {22, 23, 11, 6};
const int IN1[4] = {24, 25, 30, 32};
const int IN2[4] = {26, 27, 31, 33};

// Claw servo or motor pin
const int CLAW_PIN = 7;

// ================= CONTROL =================
bool manualMode = false;
const int Kp = 1; // Simple proportional gain for arm control

// ================= SETUP =================
void setup()
{
        Serial.begin(115200);

        for (int i = 0; i < 4; i++)
        {
                pinMode(encA[i], INPUT);
                pinMode(encB[i], INPUT);
                pinMode(EN[i], OUTPUT);
                pinMode(IN1[i], OUTPUT);
                pinMode(IN2[i], OUTPUT);
        }

        pinMode(CLAW_PIN, OUTPUT);

        attachInterrupt(digitalPinToInterrupt(encA[0]), isr0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encA[1]), isr1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encA[2]), isr2, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encA[3]), isr3, CHANGE);
}

// ================= LOOP =================
void loop()
{
        if (Serial.available())
        {
                String cmd = Serial.readStringUntil('\n');
                parseCommand(cmd);
        }

        // If not in manual mode, apply proportional control to arms
        if (!manualMode)
        {
                for (int i = ARM_1; i <= ARM_2; i++)
                {
                        long error = targetCount[i] - encCount[i];
                        int speed = Kp * error;
                        speed = constrain(speed, -255, 255);
                        moveMotor(i, speed);
                }
        }
}

// ================= SERIAL COMMAND PARSER =================
void parseCommand(String cmd)
{
        cmd.trim();

        if (cmd.startsWith("DRIVE"))
        {
                manualMode = false; // auto commands disable manual override
                int l, r;
                sscanf(cmd.c_str(), "DRIVE %d %d", &l, &r);
                moveMotor(DRIVE_L, l);
                moveMotor(DRIVE_R, r);
        }
        else if (cmd.startsWith("ARM_POS"))
        {
                manualMode = false;
                int joint, ticks;
                sscanf(cmd.c_str(), "ARM_POS %d %d", &joint, &ticks);
                if (joint == 1)
                        targetCount[ARM_1] = ticks;
                if (joint == 2)
                        targetCount[ARM_2] = ticks;
        }
        else if (cmd.startsWith("ARM_GRAB"))
        {
                String state = cmd.substring(9);
                if (state == "CLOSE")
                        digitalWrite(CLAW_PIN, HIGH);
                else if (state == "OPEN")
                        digitalWrite(CLAW_PIN, LOW);
        }
        else if (cmd == "STOP")
        {
                stopAll();
        }
        else if (cmd == "STATUS")
        {
                sendStatus();
        }
        else if (cmd == "MANUAL_ON")
        {
                manualMode = true;
        }
        else if (cmd == "MANUAL_OFF")
        {
                manualMode = false;
        }
        else if (manualMode && cmd.startsWith("MANUAL"))
        {
                // Example: MANUAL DRIVE 150 -150 or MANUAL ARM1 120
                if (cmd.indexOf("DRIVE") >= 0)
                {
                        int l, r;
                        sscanf(cmd.c_str(), "MANUAL DRIVE %d %d", &l, &r);
                        moveMotor(DRIVE_L, l);
                        moveMotor(DRIVE_R, r);
                }
                else if (cmd.indexOf("ARM1") >= 0)
                {
                        int s;
                        sscanf(cmd.c_str(), "MANUAL ARM1 %d", &s);
                        moveMotor(ARM_1, s);
                }
                else if (cmd.indexOf("ARM2") >= 0)
                {
                        int s;
                        sscanf(cmd.c_str(), "MANUAL ARM2 %d", &s);
                        moveMotor(ARM_2, s);
                }
        }
}

// ================= ENCODER ISRs =================
void isr0() { updateEncoder(0); }
void isr1() { updateEncoder(1); }
void isr2() { updateEncoder(2); }
void isr3() { updateEncoder(3); }

void updateEncoder(int i)
{
        int a = digitalRead(encA[i]);
        int b = digitalRead(encB[i]);
        encDir[i] = (a == HIGH) ? (b == HIGH) : encDir[i];
        if (encDir[i])
                encCount[i]++;
        else
                encCount[i]--;
}

// ================= MOTOR CONTROL =================
void moveMotor(int i, int speedVal)
{
        if (speedVal >= 0)
        {
                digitalWrite(IN1[i], HIGH);
                digitalWrite(IN2[i], LOW);
        }
        else
        {
                digitalWrite(IN1[i], LOW);
                digitalWrite(IN2[i], HIGH);
                speedVal = -speedVal;
        }
        analogWrite(EN[i], constrain(speedVal, 0, 255));
}

void stopAll()
{
        for (int i = 0; i < 4; i++)
                analogWrite(EN[i], 0);
}

// ================= STATUS =================
void sendStatus()
{
        Serial.print("ENC ");
        for (int i = 0; i < 4; i++)
        {
                Serial.print(encCount[i]);
                Serial.print(" ");
        }
        Serial.println();
}
