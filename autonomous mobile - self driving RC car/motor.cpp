/*
 * motor.cpp
 *
 *  Created on: 2015¦~11¤ë18¤é
 *      Author: YuYu
 */

#include "motor.hpp"
#include <stdio.h>
#include "lpc_pwm.hpp"
#include "utilities.h"
#include "io.hpp"
#include <stdint.h>



motorTask::motorTask(uint8_t priority) :
        scheduler_task("motor_servo", 512 * 4, priority), motor(PWM::pwm1, 100), servo(
                PWM::pwm2, 100)
{
    static int initESC = 0;

    while (initESC < 3)
    {
        delay_ms(300); // need 3 pulses for initializing ESC, Preet's function
        motor.set(15); // neutral
        initESC++;
    }

}

bool motorTask::run(void *p)
{

    steering_t steer;
    motor_t moveForward;

    QueueHandle_t steeringQueue = getSharedObject(shared_steeringQ);
    QueueHandle_t motorQueue = getSharedObject(shared_motorQ);

    if (xQueueReceive(steeringQueue, &steer, 1500))
        {
        switch(steer){
            case(0): puts("slight right"); turnRight(14); break;
            case(1): puts("max_right"); turnRight(12.5); break;
            case(2): puts("slight left");turnLeft(16); break;
            case(3): puts("max left"); turnLeft(17.5); break;
            case(4): puts("forward"); resetAll(15); break;
            default: puts("stop"); resetAll(15); break;
            }
        }
        else
            puts("STEERING FAILED");

    if(xQueueReceive(motorQueue, &moveForward, 1500)){
        switch(moveForward){
            case(0): puts("motor stopped"); motorForward(15);  break;
            case(1): puts("motor movin forward"); motorForward(16.5);  break;
            default: puts("no case of motor"); break;
        }
    }
    /*
    if (SW.getSwitch(1))
    { // switch 1
        motorForward();
        puts("forward");
        //motor.set(16);
    }
    else if(SW.getSwitch(2))
    { // switch 2
      //resetAll();
      //resetAll();
        motorBackward();
        puts("backwards");
      //motor.set(13.5);
    }
    if(SW.getSwitch(4))
    {
        turnRight();
        puts("right");
        //servo.set(10.5);
    }
    else if(SW.getSwitch(3))
    {
        turnLeft();
        puts("left");
        //servo.set(19.5);
    }
    if(SW.getSwitch(1) && SW.getSwitch(4))
    {
        resetAll();
        puts("reset");
        //motor.set(15);
        //servo.set(15);
    }
    */
    //vTaskDelay(1000);
    return true;
}

void motorTask::resetAll(float val)
{
    //motor.set(val); // neutral 15
    servo.set(val); // center15

    return;
}
void motorTask::turnLeft(float val)
{
    servo.set(val); // left , 19.5 max
    return;
}
void motorTask::turnRight(float val)
{
    servo.set(val); //rights 10.5 max
    return;
}
void motorTask::motorForward(float val)
{
    motor.set(val); // 15% motor speed, 16.5
    return;

}
void motorTask::motorBackward(float val)
{
    motor.set(val); //13.5
    return;
}

