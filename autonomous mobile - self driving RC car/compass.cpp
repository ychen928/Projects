/*
 * compass.cpp
 *
 *  Created on: 2015¦~11¤ë7¤é
 *      Author: YuYu
 */

#include "compass.hpp"
#include <stdio.h>
#include <math.h>
#include <i2c2.hpp>
#include "utilities.h"


compassI2C::compassI2C(uint8_t priority) :
        scheduler_task("compass", 512 * 4, priority), compass(
                I2C2::getInstance())
{
    QueueHandle_t compassQueue = xQueueCreate(1, sizeof(float));
    addSharedObject(shared_CompassQ, compassQueue);     // refer to geo.hpp

    compass.writeReg(I2CADDR_Compass, Config_regA,
            eight_samples | _15 | normal); // 8 samples, 15Hz, no bias
    printf("INIT reading register 0x00: %#2X\n",
            compass.readReg(I2CADDR_Compass, Config_regA));

    compass.writeReg(I2CADDR_Compass, Config_regB, _4_7); // gain: +- 4.7Ga
    printf("INIT reading register 0x01: %#2X\n",
            compass.readReg(I2CADDR_Compass, Config_regB));

    compass.writeReg(I2CADDR_Compass, Mode_reg, continuous); // continuous mode
    printf("INIT reading register 0x02: %#2X\n",
            compass.readReg(I2CADDR_Compass, Mode_reg));
    uint8_t buffer[6] = { 0 };
    printf("statusB: %#2X\n", compass.readReg(I2CADDR_Compass, Stat_reg));
    compass.readRegisters(0x3C,0x3,&buffer[0],sizeof(buffer)); // flush all 6 data registers to reset stat reg to 0x11
    printf("statusA: %#2X\n", compass.readReg(I2CADDR_Compass, Stat_reg));

    //calibrate();
}

bool compassI2C::run(void *p)
{
    if (SW.getSwitch(1))
        { // switch 1
            calibrate();
        }


    //printf("statusA: %#2X\n", compass.readReg(I2CADDR_Compass, Stat_reg));

    if(compass.readReg(I2CADDR_Compass, Stat_reg) == 0x11){ // if valid, compute heading
    float head = computeHeading();
    if(!xQueueSend(getSharedObject(shared_CompassQ), &head, 1000)){
            puts("FAILED TO SEND COMPASS HEADING WITHIN 1000ms, (QUEUE FULL)");
        }
    }
    else if(compass.readReg(I2CADDR_Compass, Stat_reg) == 0x3){ //re-read
        computeHeading();
    }
    vTaskDelay(1000);

    return true;
}


float compassI2C::computeHeading()
{
    //FILE *dataxyz = fopen("0:data.txt", "a");

    int16_t x = compass.readReg(I2CADDR_Compass, X_MSB) << 8
            | (compass.readReg(I2CADDR_Compass, X_LSB) & 0xFF);
    x = x_scale * (x - x_offset);


    int16_t z = compass.readReg(I2CADDR_Compass, Z_MSB) << 8
            | (compass.readReg(I2CADDR_Compass, Z_LSB) & 0xFF);
    z= z_scale * (z - z_offset);


    int16_t y = compass.readReg(I2CADDR_Compass, Y_MSB) << 8
            | (compass.readReg(I2CADDR_Compass, Y_LSB) & 0xFF);
    y = y_scale * (y - y_offset);

    float heading = (float) atan2(float(y), (float(x))); // in radian

    if (heading < 0)
    {
        heading += (2 * M_PI);
        heading *= 180 / M_PI;
    }
    else
        heading *= 180 / M_PI;

    //printf("%d           %d            %d             %f\n", x, y, z, heading);
    //fprintf(dataxyz,"%d           %d            %d            %f\n",x,y,z, heading);

    //fclose(dataxyz);

    return heading;
}

void compassI2C::calibrate()
{
    int sample = 200;
    int16_t mag_temp[3] =
    { 0 };
    int16_t mag_bias[3] =
    { 0 };
    int16_t mag_scale[3] =
    { 0 };
    int16_t mag_max[3] =
    { 0 };
    int16_t mag_min[3] =
    { 0 };

    for (int i = 0; i < sample; i++)
    {
        mag_temp[0] = compass.readReg(I2CADDR_Compass, X_MSB) << 8
                | (compass.readReg(I2CADDR_Compass, X_LSB) & 0xFF);

        mag_temp[2] = compass.readReg(I2CADDR_Compass, Z_MSB) << 8
                | (compass.readReg(I2CADDR_Compass, Z_LSB) & 0xFF);

        mag_temp[1] = compass.readReg(I2CADDR_Compass, Y_MSB) << 8
                | (compass.readReg(I2CADDR_Compass, Y_LSB) & 0xFF);

        printf("%d       %d        %d\n", mag_temp[0], mag_temp[1],
                mag_temp[2]);

        for (int jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        delay_ms(300);
        printf("sample: %d\n", i);
    }

    printf("min x: %d\n", mag_min[0]);
    printf("max x: %d\n", mag_max[0]);

    printf("min y: %d\n", mag_min[1]);
    printf("max y: %d\n", mag_max[1]);

    printf("min z: %d\n", mag_min[2]);
    printf("max z: %d\n", mag_max[2]);

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias

    printf("hard x: %d\n", mag_bias[0]);
    printf("hard y: %d\n", mag_bias[1]);
    printf("hard z: %d\n", mag_bias[2]);

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2]; //F = (fx+fy+fz)/3
    avg_rad /= 3.0;

    printf("soft x: %f\n", avg_rad / ((float) mag_scale[0])); //Scale_x = F/fx
    printf("soft y: %f\n", avg_rad / ((float) mag_scale[1])); //     _y
    printf("soft z: %f\n", avg_rad / ((float) mag_scale[2])); //     _z

}

