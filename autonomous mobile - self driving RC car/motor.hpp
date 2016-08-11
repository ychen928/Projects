/*
 * motor.hpp
 *
 *  Created on: 2015¦~11¤ë18¤é
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_MOTOR_HPP_
#define L5_APPLICATION_MOTOR_HPP_

#include "tasks.hpp"
#include "lpc_pwm.hpp"

#include "geo.hpp"

typedef enum{
    slight_right,
    max_right,
    slight_left,
    max_left,
    forward
}steering_t;

typedef enum{
    stop,
    forwardDir
}motor_t;

class motorTask: public scheduler_task
{
    public:

        motorTask(uint8_t priority); //constructor
        bool run(void *p); //
        void resetAll(float val);
        void turnLeft(float val);
        void turnRight(float val);
        void motorForward(float val);
        void motorBackward(float val);

    private:
        PWM motor; // P2.0 for PWM
        PWM servo; // P2.1, magical function that initializes everything :)

};

#endif /* L5_APPLICATION_MOTOR_HPP_ */
