#ifndef MECANUM_CONTROL_H
#define MECANUM_CONTROL_H

#include "motor_control.h"

class MecanumControl {
private:
    MotorControl motor1, motor2, motor3, motor4;
    int wheelSpeed;  // Motor hız değeri

public:
    MecanumControl(int motor1_pwm, int motor1_inA, int motor1_inB,
                   int motor2_pwm, int motor2_inA, int motor2_inB,
                   int motor3_pwm, int motor3_inA, int motor3_inB,
                   int motor4_pwm, int motor4_inA, int motor4_inB)
        : motor1(motor1_pwm, motor1_inA, motor1_inB, 0),
          motor2(motor2_pwm, motor2_inA, motor2_inB, 1),
          motor3(motor3_pwm, motor3_inA, motor3_inB, 2),
          motor4(motor4_pwm, motor4_inA, motor4_inB, 3){}
    
    void setSpeed(int speed) {
        wheelSpeed = constrain(speed, 0, 255);  // Hızı 0 ile 255 arasında sınırlıyoruz
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }
    void moveForward() {
        motor1.moveForward();
        motor2.moveForward();
        motor3.moveForward();
        motor4.moveForward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void moveBackward() {
        motor1.moveBackward();
        motor2.moveBackward();
        motor3.moveBackward();
        motor4.moveBackward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void moveSidewaysRight() {
        motor1.moveBackward();
        motor2.moveForward();
        motor3.moveForward();
        motor4.moveBackward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void moveSidewaysLeft() {
        motor1.moveForward();
        motor2.moveBackward();
        motor3.moveBackward();
        motor4.moveForward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void rotateLeft() {
        motor1.moveBackward();
        motor2.moveForward();
        motor3.moveBackward();
        motor4.moveForward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void rotateRight() {
        motor1.moveForward();
        motor2.moveBackward();
        motor3.moveForward();
        motor4.moveBackward();
        motor1.setSpeed(wheelSpeed);
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void moveRightForward() {
        motor1.moveForward();
        motor2.stop();
        motor3.stop();
        motor4.moveForward();
        motor1.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void moveRightBackward() {
        motor1.stop();
        motor2.moveBackward();
        motor3.moveBackward();
        motor4.stop();
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
    }

    void moveLeftForward() {
        motor1.stop();
        motor2.moveForward();
        motor3.moveForward();
        motor4.stop();
        motor2.setSpeed(wheelSpeed);
        motor3.setSpeed(wheelSpeed);
    }

    void moveLeftBackward() {
        motor1.moveBackward();
        motor2.stop();
        motor3.stop();
        motor4.moveBackward();
        motor1.setSpeed(wheelSpeed);
        motor4.setSpeed(wheelSpeed);
    }

    void stop() {
        motor1.stop();
        motor2.stop();
        motor3.stop();
        motor4.stop();
    }
};

#endif  // MECANUM_CONTROL_H
