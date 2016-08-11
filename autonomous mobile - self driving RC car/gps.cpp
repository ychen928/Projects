/*
 * gps.cpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#include "gps.hpp"
#include <stdio.h>

gpsTask::gpsTask(uint8_t priority) :
        scheduler_task("gps", 512 * 4, priority), gpsUart3(
                Uart3::getInstance()), latitude(37), longitude(-121)
{
    //inits
    QueueHandle_t gpsDataQueue = xQueueCreate(1, sizeof(float));
    addSharedObject(shared_GPSQ, gpsDataQueue); // refer to geo.hpp

    gpsUart3.init(9600, 1000, 10); // Baud rate, rx Queue size, tx Queue size
    gpsUart3.putline(PMTK_SET_NMEA_OUTPUT_RMCONLY, portMAX_DELAY); // output only RMC sentence
}

float lati, longi;
bool gpsTask::run(void *p)
{
    // recieve buffer, upper half is parsing NMEA RMC for latitude and longitude
    char rx[150] =
    { "\0" };
    gpsUart3.gets(rx, 150, portMAX_DELAY);

    if (strstr(rx, "$GPRMC")) // look for $GPRMC in string
    {
        /*
         for (int i = 0; i < 79; i++)
         {
         printf("%c", rx[i]);
         }
         */
        tokenize(rx); // split into tokens (parsing)
        //printf("\nlat: %f      long: %f\n", latitude, longitude);
    }
    // bottom half is for computing bearing and distance and passing it to geoTask

    float bearing, distance;
    float *GPSdata; // pointer to where data is stored

    if (latitude < MAXRANGE_LATITUDE && latitude > MINRANGE_LATITUDE
            && longitude > MAXRANGE_LONGITUDE && longitude < MINRANGE_LONGITUDE)
    {
        LE.on(4);
    }
    else
    {
        LE.toggle(4);
    }

    if (lati < MAXRANGE_LATITUDE && lati > MINRANGE_LATITUDE
             && longi > MAXRANGE_LONGITUDE && longi < MINRANGE_LONGITUDE)
     {
         LE.on(3);
     }
     else
     {
         LE.toggle(3);
     }

    bearing = computeBearing(latitude, longitude, lati, longi );
    distance = computeDistance(latitude, longitude, lati, longi );
    results[0] = bearing;
    results[1] = distance;

    //printf("bearing: %f         distance: %f\n", results[0], results[1]);

    GPSdata = results;

    if (!xQueueSend(getSharedObject(shared_GPSQ), &GPSdata, 1000))
    {
        puts("FAILED TO SEND GPS DATA WITHIN 1000ms, QUEUE FULL");
        LE.on(2);
    }
    else
    {
        LE.off(2);
    }

    vTaskDelay(1000);
    return true;
}

float gpsTask::computeBearing(float gps_lat, float gps_long, float dst_lat,
        float dst_long)
{
    // bearing = atan2( sin(longDiff_rad)*cos(lat2_rad),
    //cos(lat1_rad)*sin(lat2_rad)-sin(lat1_rad)*cos(lat2_rad)*cos(longDiff_rad))

    float lat1_rad = toRadian(gps_lat);
    float lat2_rad = toRadian(dst_lat);
    float longDiff_rad = toRadian(dst_long - gps_long);

    float y = sin(longDiff_rad) * cos(lat2_rad);
    float x = cos(lat1_rad) * sin(lat2_rad)
            - sin(lat1_rad) * cos(lat2_rad) * cos(longDiff_rad);
    float bearing = fmodf(toDegree(atan2(y, x)) + 360, 360);

    return bearing;
}

float gpsTask::computeDistance(float gps_lat, float gps_long, float dst_lat,
        float dst_long)
{
    //haversine formula: a = sin^2(latDiff_rad/2) + cos(lat1_rad) * cos(lat2_rad) * sin^2(longDiff_rad/2)
    // formula: c = 2 *atan2(sqrt(a), sqrt(1-a))
    // d = R*c

    float earth_radius = 6371000; //meters
    float lat1_rad = toRadian(gps_lat);
    float lat2_rad = toRadian(dst_lat);
    float latDiff_rad = toRadian(dst_lat - gps_lat);
    float longDiff_rad = toRadian(dst_long - gps_long);

    float a = sin(latDiff_rad / 2) * sin(latDiff_rad / 2)
            + cos(lat1_rad) * cos(lat2_rad) * sin(longDiff_rad / 2)
                    * sin(longDiff_rad / 2);

    float c = 2.0 * atan2(sqrt(a), sqrt(1 - a));
    float d = earth_radius * c;

    return d;
}

void gpsTask::tokenize(char *line)
{
    const char delimiter[2] = _delimiter;
    char *token;
    int counter = 0;

    token = strtok(line, delimiter);
    float latTemp = latitude;
    float longTemp = longitude;
    while (token != NULL)
    {
        if (counter == 3)
        {/*convert the 3trd token latitude*/
            latTemp = atof(token);
            latTemp = convertFromNmeaSentenceToDecimalCoord(latTemp, "m"); //"m" for meridian
        }
        /*If 5th token == South multiply by -1*/
        if (counter == 4)
        {
            if (*token == 'S')
            {
                latTemp *= (-1);
            }
        }
        latitude = latTemp;
        /*convert the 5th token longitude*/
        if (counter == 5)
        {
            longTemp = atof(token);
            longTemp = convertFromNmeaSentenceToDecimalCoord(longTemp, "p"); //"p" for parallel
        }
        /*If 6th token == West multiply by -1*/
        if (counter == 6)
        {
            if (*token == 'W')
            {
                longTemp *= (-1);
            }
        }
        longitude = longTemp;
        token = strtok(NULL, delimiter);
        ++counter;
    }
}

float gpsTask::convertFromNmeaSentenceToDecimalCoord(float coordinates,
        const char *val)
{
    /* Sample from gps 5153.6605*/
    /* Check limits*/
    if ((*val == 'm') && (coordinates < 0.0 && coordinates > MAX_LATITUDE))
    {
        return 0;
    }
    if (*val == 'p' && (coordinates < 0.0 && coordinates > MAX_LONGITUDE))
    {
        return 0;
    }
    int b; //to store the degrees
    float c; //to store de decimal

    /*Calculate the value in format nn.nnnnnn*/
    /*Explanations at:
     http://www.mapwindow.org/phorum/read.php?3,16271,16310*/

    b = coordinates / 100; // 51 degrees
    c = (coordinates / 100 - b) * 100; //(51.536605 - 51)* 100 = 53.6605
    c /= 60; // 53.6605 / 60 = 0.8943417
    c += b; // 0.8943417 + 51 = 51.8943417
    return c;
}

float gpsTask::toRadian(float degree)
{
    return degree * (M_PI / 180.0);
}

float gpsTask::toDegree(float radian)
{
    return radian * (180.0 / M_PI);
}
