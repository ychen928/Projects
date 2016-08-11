/*
 * compass.hpp
 *
 *  Created on: 2015¦~11¤ë7¤é
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_COMPASS_HPP_
#define L5_APPLICATION_COMPASS_HPP_

#include <i2c2.hpp>
#include "tasks.hpp"
#include "geo.hpp"
#include "io.hpp"


#define I2CADDR_Compass      0x3C
#define x_offset            20//29//33//21//-2 // car, no car
#define y_offset            -35//-41//-31//-51//-95// car, no car
#define z_offset            0//-43
#define x_scale             1//0.846405
#define y_scale             1//0.799383
#define z_scale             1//1.761905
/*
 no car
 hard x: -2
 hard y: -95
 hard z: -43
 soft x: 0.362179
 soft y: 4.185185

 car
 hard x: 21
 hard y: -54
 hard z: -51
 soft x: 0.816199
 soft y: 0.839744
 soft z: 1.712418


 */
typedef enum
{
    Config_regA = 0x0,
    Config_regB = 0x1,
    Mode_reg = 0x2,
    X_MSB = 0x3,
    X_LSB = 0x4,
    Z_MSB = 0x5,
    Z_LSB = 0x6,
    Y_MSB = 0x7,
    Y_LSB = 0x8,
    Stat_reg = 0x9
} Register_list;

typedef enum
{
    one_sample = (0x0 << 5),
    two_samples = (0x1 << 5),
    four_samples = (0x2 << 5),
    eight_samples = (0x3 << 5)
} avg_sample; //configA_reg, samples averaged/ measurement output

typedef enum
{
    _0_75 = (0x0 << 2),
    _1_5 = (0x1 << 2),
    _3 = (0x2 << 2),
    _7_5 = (0x3 << 2),
    _15 = (0x4 << 2),
    _30 = (0x5 << 2),
    _75 = (0x6 << 2)
} DoutRate; // configA_reg, Data output rate bits, rate at which data is written to all three data output register

typedef enum
{
    normal = (0x0 << 0),
    positive = (0x1 << 0),
    negative = (0x2 << 0)
} Measurement_Bias; // configA_reg, measurement flow of device, incorporate an applied bias into measurement

typedef enum
{
    __88 = (0x0 << 5),
    _1_3 = (0x1 << 5),
    _1_9 = (0x2 << 5),
    _2_5 = (0x3 << 5),
    _4 = (0x4 << 5),
    _4_7 = (0x5 << 5),
    _5_6 = (0x6 << 5),
    _8_1 = (0x7 << 5)
} gain; //configB_reg, configure gain bits, +-Ga, _ means decimal point between number

typedef enum
{
    continuous = (0x0 << 0),
    single = (0x1 << 0)
} operation_mode; //mode_reg, mode of operation

class compassI2C: public scheduler_task
{
    public:
        compassI2C(uint8_t priority); //constructor
        bool run(void *p); //
        float computeHeading(); // computes and returns heading
        void calibrate(); // calibration function for hard & soft iron based on 200 samples

    private:
        I2C2& compass;

};

#endif /* L5_APPLICATION_COMPASS_HPP_ */
