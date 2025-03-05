#ifndef MECANUM_CONTROL_H
#define MECANUM_CONTROL_H

#include "motor_control.h"

class MecanumControl {
private:
    MotorControl motor1, motor2, motor3, motor4;
    float wheel_radius = 0.05; // Tekerlek yarıçapı (m)
    float robot_length = 0.2; // Robotun uzunluğu / 2 (m)
    float robot_width = 0.15; // Robotun genişliği / 2 (m)

public:
    MecanumControl(int motor1_pwm, int motor1_inA, int motor1_inB,
                   int motor2_pwm, int motor2_inA, int motor2_inB,
                   int motor3_pwm, int motor3_inA, int motor3_inB,
                   int motor4_pwm, int motor4_inA, int motor4_inB)
        : motor1(motor1_pwm, motor1_inA, motor1_inB, 0),
          motor2(motor2_pwm, motor2_inA, motor2_inB, 1),
          motor3(motor3_pwm, motor3_inA, motor3_inB, 2),
          motor4(motor4_pwm, motor4_inA, motor4_inB, 3) {}
    
    void moveWithCmdVel(float Vx, float Vy, float omega) {
        float L = robot_length;
        float W = robot_width;
        float r = wheel_radius;

        // Calculate the motor speeds
        float W1 = (Vx - Vy - (L + W) * omega) / r;
        float W2 = (Vx + Vy + (L + W) * omega) / r;
        float W3 = (Vx - Vy + (L + W) * omega) / r;
        float W4 = (Vx + Vy - (L + W) * omega) / r;

        // Set the motor speeds
        setMotorSpeed(motor1, W1);
        setMotorSpeed(motor2, W2);
        setMotorSpeed(motor3, W3);
        setMotorSpeed(motor4, W4);
    }
    
    void setMotorSpeed(MotorControl &motor, float speed) {
		    motor.setSpeed(constrain(abs(speed) * 255, 0, 255));
		    if (speed > 0) {
		        motor.moveForward();
		    } else if (speed < 0) {
		        motor.moveBackward();
		    } else {
		        motor.stop();
		    }
}

};

#endif  // MECANUM_CONTROL_H
