/*
 * gps.hpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_GPS_HPP_
#define L5_APPLICATION_GPS_HPP_

#include "tasks.hpp"
#include "geo.hpp"
#include "uart3.hpp"
#include <string.h>
#include <math.h>
#include "io.hpp"

#define MINRANGE_LONGITUDE  -120    // range of location in bay area, california
#define MAXRANGE_LONGITUDE  -123

#define MINRANGE_LATITUDE  36
#define MAXRANGE_LATITUDE  38

#define MAX_LONGITUDE 180
#define MAX_LATITUDE   90
#define _delimiter   ","
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

class gpsTask: public scheduler_task
{
    public:
        gpsTask(uint8_t priority); //constructor
        bool run(void *p); //
        float computeBearing(float gps_lat, float gps_long, float dst_lat, float dst_long); // calculate bearing
        float computeDistance(float gps_lat, float gps_long, float dst_lat, float dst_long); // calculate distance
        void tokenize(char *line);  // parse RMC sentence for latitude and longitude
        float convertFromNmeaSentenceToDecimalCoord(float coordinates, const char *val); // self-explanatory
        float toRadian(float degree);
        float toDegree(float radian);


    private:
        Uart3& gpsUart3;
        float latitude;
        float longitude;
        float results[2] = {0}; // [0] = bearing  ,  [1] = distance

};



#endif /* L5_APPLICATION_GPS_HPP_ */
