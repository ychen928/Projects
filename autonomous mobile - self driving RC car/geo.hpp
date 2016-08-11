/*
 * geo.hpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_GEO_HPP_
#define L5_APPLICATION_GEO_HPP_

#include "tasks.hpp"
#include "motor.hpp"
#include "io.hpp"

typedef enum{
    shared_CompassQ,
    shared_GPSQ,
    shared_steeringQ,
    shared_motorQ,
    shared_bluetoothQ
}sharedHandleID_t;



class geoTask: public scheduler_task       //This task combines both the compass and gps modules
{
    public:
        geoTask(uint8_t priority); //constructor
        bool run(void *p); //


    private:


};



#endif /* L5_APPLICATION_GEO_HPP_ */
