/*
 * geo.cpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#include "geo.hpp"
#include <stdio.h>

geoTask::geoTask(uint8_t priority) :
        scheduler_task("geo", 512 * 4, priority)
{
    //inits
    QueueHandle_t steeringQueue = xQueueCreate(1, sizeof(steering_t));
    addSharedObject(shared_steeringQ, steeringQueue); // refer to geo.hpp

    QueueHandle_t motorQueue = xQueueCreate(1, sizeof(motor_t));
    addSharedObject(shared_motorQ, motorQueue); // refer to geo.hpp

}

bool geoTask::run(void *p)
{
    float head;
    float *GPS;
    float difference;
    steering_t steer;
    motor_t moveForward;

    QueueHandle_t compassQueue = getSharedObject(shared_CompassQ);
    QueueHandle_t gpsDataQueue = getSharedObject(shared_GPSQ);

    if (xQueueReceive(compassQueue, &head, 1000))
    {
        printf("heading FROM GEO CLASS: %f\n", head);
        LE.off(1);
    }
    else{
        puts("HEADING FAILED");
        LE.on(1);
    }
    if (xQueueReceive(gpsDataQueue, &GPS, 1000)) //GPS[0] = bearing, GPS[1] = distance
    {
        printf("bearing FROM GEO CLASS: %f   %f\n", GPS[0], GPS[1]);
    }
    else
        puts("GPS FAILED");

    difference = head - GPS[0];
    //printf("DIFFERENCE                  %f\n", difference);
    // the bearing-heading difference will make car steer toward dst

    if (difference <= -180)
        steer = max_left; //  3
    else if (difference >= 180)
        steer = max_right; //  1
    else if (difference > -180 && difference < -15)
        steer = slight_right; //  0
    else if (difference < 180 && difference > 15)
        steer = slight_left; //  2
    else
        steer = forward; //  4

    if (GPS[1] >5)
        moveForward = forwardDir;
    else
        moveForward = stop;

    //printf("steer: %i\n", steer);

    if (!xQueueSend(getSharedObject(shared_steeringQ), &steer, 1000))
    {
        puts(
                "FAILED TO SEND DIFFERENCE DATA TO MOTOR TASK WITHIN 1000ms, QUEUE FULL");
    }

    if (!xQueueSend(getSharedObject(shared_motorQ), &moveForward, 1000))
    {
        puts(
                "FAILED TO SEND moveforward DATA TO MOTOR TASK WITHIN 1000ms, QUEUE FULL");
    }

    vTaskDelay(1000);

    return true;
}
