/*
 * CAN.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_CAN_HPP_
#define L5_APPLICATION_CAN_CAN_HPP_

#include "mcp2515.hpp"

#define CANSPEED_33     14      // CAN speed at 33 kbps
#define CANSPEED_50     9       // CAN speed at 50 kbps
#define CANSPEED_83     5       // CAN speed at 83 kbps
#define CANSPEED_100    4       // CAN speed at 100 kbps
#define CANSPEED_125    3       // CAN speed at 125 kbps
#define CANSPEED_166    2       // CAN speed at 166 kbps
#define CANSPEED_250    1       // CAN speed at 250 kbps
#define CANSPEED_500    0       // CAN speed at 500 kbps

#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE            0x11

#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8


/*
 * More C++ Wrapper
 * Does the sending/receiving of messages and get Car Velocity in Km/Hr
 * */
class CANbus {
    public:
        CANbus(char speed);

        bool CAN_sendMessage(char pid);
        bool CAN_getMessage( char *messageBuffer);
        char getSpeed(char pid);


    private:
        mcp2515 controller;
};

#endif /* L5_APPLICATION_CAN_CAN_HPP_ */
