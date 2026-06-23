#include <Arduino.h>
#include <ESP32Servo.h>

// Servo pins
#define SERVO_J1_PIN  13   // base rotation
#define SERVO_J2_PIN  12   // shoulder
#define SERVO_J3_PIN  14   // elbow
#define SERVO_J4_PIN  27   // wrist
#define SERVO_GRIP_PIN 26  // gripper

// Servo travel limits (degrees)
#define SERVO_MIN_DEG  0
#define SERVO_MAX_DEG  180
#define SERVO_MID_DEG  90

// Gripper positions
#define GRIP_OPEN_DEG   60
#define GRIP_CLOSE_DEG  120

// Serial receive buffer
#define BUF_SIZE 64
char rxBuf[BUF_SIZE];
uint8_t rxIdx = 0;

Servo servos[5];
const uint8_t SERVO_PINS[5] = {
    SERVO_J1_PIN, SERVO_J2_PIN, SERVO_J3_PIN, SERVO_J4_PIN, SERVO_GRIP_PIN
};

// Current servo positions
float currentAngles[4] = {90, 90, 90, 90};
const float SLEW_RATE = 2.0;  // max degrees per tick (smooth motion)

// Target angles (from ROS2)
float targetAngles[4] = {90, 90, 90, 90};
bool gripClose = false;

void moveTowardsTarget() {
    for (int i = 0; i < 4; i++) {
        float diff = targetAngles[i] - currentAngles[i];
        if (abs(diff) < SLEW_RATE) {
            currentAngles[i] = targetAngles[i];
        } else {
            currentAngles[i] += (diff > 0) ? SLEW_RATE : -SLEW_RATE;
        }
        // Clamp and write
        float clamped = constrain(currentAngles[i], SERVO_MIN_DEG, SERVO_MAX_DEG);
        servos[i].write((int)clamped);
    }
}

void parseJointCommand(const char* cmd) {
    // Format: "j1,j2,j3,j4\n"  (degrees, float)
    float a0, a1, a2, a3;
    if (sscanf(cmd, "%f,%f,%f,%f", &a0, &a1, &a2, &a3) == 4) {
        // Convert from robot-frame degrees to servo degrees
        // J1: centre at 90deg, positive = CCW
        targetAngles[0] = constrain(90.0f + a0, SERVO_MIN_DEG, SERVO_MAX_DEG);
        targetAngles[1] = constrain(90.0f + a1, SERVO_MIN_DEG, SERVO_MAX_DEG);
        targetAngles[2] = constrain(90.0f + a2, SERVO_MIN_DEG, SERVO_MAX_DEG);
        targetAngles[3] = constrain(90.0f + a3, SERVO_MIN_DEG, SERVO_MAX_DEG);
        Serial.println("OK");
    } else {
        Serial.println("ERR:PARSE");
    }
}

void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            rxBuf[rxIdx] = '\0';
            rxIdx = 0;

            if (strncmp(rxBuf, "GRIP_CLOSE", 10) == 0) {
                servos[4].write(GRIP_CLOSE_DEG);
                Serial.println("OK:GRIP_CLOSE");
            } else if (strncmp(rxBuf, "GRIP_OPEN", 9) == 0) {
                servos[4].write(GRIP_OPEN_DEG);
                Serial.println("OK:GRIP_OPEN");
            } else if (strncmp(rxBuf, "HOME", 4) == 0) {
                for (int i = 0; i < 4; i++) targetAngles[i] = SERVO_MID_DEG;
                Serial.println("OK:HOME");
            } else {
                parseJointCommand(rxBuf);
            }
        } else {
            if (rxIdx < BUF_SIZE - 1) rxBuf[rxIdx++] = c;
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Attach servos
    for (int i = 0; i < 5; i++) {
        servos[i].attach(SERVO_PINS[i]);
    }

    // Move to home
    for (int i = 0; i < 4; i++) {
        servos[i].write(SERVO_MID_DEG);
        currentAngles[i] = SERVO_MID_DEG;
        targetAngles[i]  = SERVO_MID_DEG;
    }
    servos[4].write(GRIP_OPEN_DEG);

    Serial.println("ARM_READY");
}

void loop() {
    processSerial();
    moveTowardsTarget();
    delay(20);  // 50Hz motion update
}
