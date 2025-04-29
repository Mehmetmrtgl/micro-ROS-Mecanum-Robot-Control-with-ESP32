#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    int pwmPin, inDirPin;
    int channel;

    MotorControl(int pwmPin, int inDirPin, int channel) 
        : pwmPin(pwmPin), inDirPin(inDirPin), channel(channel) {
        pinMode(inDirPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        ledcSetup(channel, 5000, 8);  // 1 kHz, 8-bit çözünürlük
        ledcAttachPin(pwmPin, channel);
    }

    void setSpeed(int speed) {
        ledcWrite(channel, speed);
    }

    void moveForward() {
        digitalWrite(inDirPin, HIGH);
    }

    void moveBackward() {
        digitalWrite(inDirPin, LOW);
    }

    void stop() {
        ledcWrite(channel, 0);  // Motoru durdur
    }
};

#endif  // MOTOR_CONTROL_H
