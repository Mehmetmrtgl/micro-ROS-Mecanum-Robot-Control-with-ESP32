#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    int pwmPin, inAPin, inBPin;
    int channel;

    MotorControl(int pwmPin, int inAPin, int inBPin, int channel) 
        : pwmPin(pwmPin), inAPin(inAPin), inBPin(inBPin), channel(channel) {
        pinMode(inAPin, OUTPUT);
        pinMode(inBPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        ledcSetup(channel, 1000, 8);  // 1 kHz, 8-bit çözünürlük
        ledcAttachPin(pwmPin, channel);
    }

    void setSpeed(int speed) {
        ledcWrite(channel, speed);
    }

    void moveForward() {
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
    }

    void moveBackward() {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, HIGH);
    }

    void stop() {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, LOW);
        ledcWrite(channel, 0);  // Motoru durdur
    }
};

#endif  // MOTOR_CONTROL_H
