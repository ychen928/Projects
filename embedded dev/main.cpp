/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "task.h"
#include "examples/examples.hpp"
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <i2c2.hpp>
#include "uart2.hpp"
#include <string>
#include "io.hpp"
#include "lpc_pwm.hpp"
#include "utilities.h"
#include <math.h>
#include <uart0_min.h>
#include "eint.h"
#include "soft_timer.hpp"
#include "event_groups.h"
#include "ssp1.h"
#include "gpio.hpp"


#include "compass.hpp"
#include "gps.hpp"
#include "motor.hpp"
#include "geo.hpp"
#include "bluetoothUART.hpp"

#include "CAN/CANTask.hpp"

//#include "own_driver/My_spi1.hpp"
//#include "own_driver/my_gpio.hpp"

using namespace std;

#define flash_manID_trans_size 5
#define flash_statReg_trans_size 2
#define mask 0xF
#define expectedbit5_2 0xB
#define BAUD_RATE 38400

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */

class GPIOTask: public scheduler_task
{
    public:
        GPIOTask(uint8_t priority) :
                scheduler_task("GPIOO", 512 * 4, priority)
        {
            // inits
        }

        bool init(void)
        {
            LPC_PINCON->PINSEL2 &= ~(3 << 0); // pin mux set to GPIO Port 1.0
            LPC_GPIO1->FIODIR |= (1 << 0); // set GPIO Port 1.0 on board LED as output

            LPC_PINCON->PINSEL2 &= ~(3 << 18); // pin mux set to GPIO Port 1.9
            LPC_GPIO1->FIODIR &= ~(1 << 9); // set GPIO Port 1.9 on board switch as input

            LPC_PINCON->PINSEL4 &= ~(3 << 6); // pin mux set to GPIO Port 2.3
            LPC_GPIO2->FIODIR |= (1 << 3); // set GPIO Port 2.3 as external LED output

            LPC_PINCON->PINSEL4 &= ~(3 << 12); // pin mux set to GPIO Port 2.6
            LPC_GPIO2->FIODIR &= ~(1 << 6); // set GPIO Port 2.6 as external switch input

            return true;
        }

        bool run(void *p)
        {

            if (LPC_GPIO2->FIOPIN & (1 << 6))
            {
                LPC_GPIO2->FIOSET = (1 << 3);
            }
            else
                LPC_GPIO2->FIOCLR = (1 << 3);
            /*
             if(LPC_GPIO1->FIOPIN & (1<<9)){  // check value of FIOPIN register, check if switch pressed
             LPC_GPIO1->FIOCLR = (1<<0);
             }else
             LPC_GPIO1->FIOSET = (1<<0);
             */
            vTaskDelay(500);
            return true;
        }
    private:
};

class SPITask: public scheduler_task
{

    public:
        SPITask(uint8_t priority) :
                scheduler_task("SPItask", 512 * 4, priority)
        {
            // inits
        }
        bool init()
        {
            LPC_PINCON->PINSEL0 &= ~(3 << 12); //P0.6 GPIO

            LPC_PINCON->PINSEL0 &= ~(3 << 14); // P0.7
            LPC_PINCON->PINSEL0 |= (2 << 14); // SCLK

            LPC_PINCON->PINSEL0 &= ~(3 << 16); // P0.8
            LPC_PINCON->PINSEL0 |= (2 << 16); // MISO

            LPC_PINCON->PINSEL0 &= ~(3 << 18); // P0.9
            LPC_PINCON->PINSEL0 |= (2 << 18); // MOSI

            LPC_GPIO0->FIODIR |= (1 << 6); // GPIO as output
            set_CSn_inactive(6);

            LPC_SC->PCONP |= (1 << 10); // power for SSP1
            LPC_SC->PCLKSEL0 &= ~(3 << 20); // precaution: clear clock bits
            LPC_SC->PCLKSEL0 |= (1 << 20); // set clock/1, in table 42

            LPC_SSP1->CR0 = 0x7; // 8 bits of data
            LPC_SSP1->CR1 = 0x2; // LBM =0, SSE = 1, MS=0

            LPC_SSP1->CPSR = 0x4; // SCK speed = CPU/4

            return true;
        }

        bool run(void *p)
        {
            char* myIDData, *myStatData, *myPage0Data;

            set_CSn_active(6);
            ssp_exchange(0x9F, 1); // ManID opcode
            myIDData = ssp_exchange(0x0, flash_manID_trans_size); // dummy bytes
            set_CSn_inactive(6);

            for (int i = 0; i < flash_manID_trans_size; i++)
            {
                if (i == 0)
                    printf("Manufacturer ID values of Flash Device: ");
                printf("%x ", myIDData[i]);
            }
            puts("");
            delete[] myIDData;

            set_CSn_active(6);
            ssp_exchange(0xD7, 1); // status Reg opcode
            myStatData = ssp_exchange(0x0, flash_statReg_trans_size); // dummy bytes
            set_CSn_inactive(6);

            for (int i = 0; i < flash_statReg_trans_size; i++)
            {
                if (i == 0)
                    puts("Status register values of Flash Device: ");
                printf("Byte %d 0x%x\n", i + 1, myStatData[i]);
                if (i == 0)
                {
                    if (myStatData[i] & (1 << 7))
                        puts("bit 7: Device is ready");
                    else
                        puts("bit 7: Device busy");
                    if (myStatData[i] & (1 << 6))
                        puts(
                                "bit 6:Main mem page data doesn't match buffer data");
                    else
                        puts("bit 6: Main mem page data matches buffer");
                    if (((myStatData[i] >> 2) & mask) == expectedbit5_2)
                        puts("bit[5:2] : 16-Mbit");
                    else
                        puts("bit[5:2] : not 16-Mbit");
                    if (myStatData[i] & (1 << 1))
                        puts("bit 1: Sector protection is enabled");
                    else
                        puts("bit 1: Sector protection is disabled");
                    if (myStatData[i] & (1 << 0))
                        puts("bit 0: Page size: 512 bytes");
                    else
                        puts("bit 0: Page size: 528 bytes");
                }
                if (i == 1)
                {
                    if (myStatData[i] & (1 << 7))
                        puts("bit 7: Device is ready");
                    else
                        puts("bit 7: Device busy");
                    if (!(myStatData[i] & (1 << 6)))
                        puts("bit 6: Reserved");
                    else
                        puts("bit 6: Error");
                    if (myStatData[i] & (1 << 5))
                        puts("bit 5: Erase or program error");
                    else
                        puts("bit 5: Erase or program operation successful");
                    if (!(myStatData[i] & (1 << 4)))
                        puts("bit 4:  Reserved");
                    else
                        puts("bit 4: Error");
                    if (myStatData[i] & (1 << 3))
                        puts("bit 3: Sector Lockdown enabled");
                    else
                        puts("bit 3: Sector Lockdown disabled");
                    if (myStatData[i] & (1 << 2))
                        puts(
                                "bit 2:  Sector in program suspended while using buffer 2");
                    else
                        puts(
                                "bit 2: No program operation suspended while using buffer 2");
                    if (myStatData[i] & (1 << 1))
                        puts(
                                "bit 1: Sector in program suspended while using buffer 1");
                    else
                        puts(
                                "bit 1: No program operation suspended while using buffer 1");
                    if (myStatData[i] & (1 << 0))
                        puts("bit 0: A sector is erase suspended");
                    else
                        puts("bit 0: No sectors are erase suspended");
                }
            }
            puts("");
            delete[] myStatData;

            set_CSn_active(6);
            ssp_exchange(0xD2, 1); // opcode for reading 0xD2
            ssp_exchange(0x0, 3); // address & page, 3 bytes
            ssp_exchange(0x0, 4); // dummy bytes, 4 bytes
            myPage0Data = ssp_exchange(0x0, 512); // read all 512 bytes of Page 0
            set_CSn_inactive(6);

            int bytes_per_sector = (myPage0Data[12] << 8) | myPage0Data[11];
            int sector_per_cluster = myPage0Data[13];
            int total_num_of_sector = (myPage0Data[20] << 8) | myPage0Data[19];
            int sig = (myPage0Data[511] << 8) | myPage0Data[510];

            printf("Bytes per sector: %d\n", bytes_per_sector);
            printf("Sector per cluster: %d\n", sector_per_cluster);
            printf("Total number of sector: %d\n", total_num_of_sector);
            printf("Signature: 0x%x\n", sig);

            puts("");
            delete[] myPage0Data;

            vTaskDelay(1000);

            return true;
        }
        bool set_CSn_inactive(int pin)
        {
            LPC_GPIO0->FIOSET = (1 << pin); // initially CSn inactive
            return true;
        }

        bool set_CSn_active(int pin)
        {
            LPC_GPIO0->FIOCLR = (1 << pin);
            return true;
        }

        char* ssp_exchange(char byte, int num_of_bytes)
        {
            char* storeData = new char[num_of_bytes];
            if (storeData == nullptr)
                puts("Fail to allocate memory");
            int counter = 0;
            while (counter != num_of_bytes)
            {
                LPC_SSP1->DR = byte;
                while (LPC_SSP1->SR & (1 << 4))
                    ;
                storeData[counter] = LPC_SSP1->DR;
                counter++;
            }
            return storeData;
        }

    private:

};

class UARTTask: public scheduler_task
{

    public:
        UARTTask(uint8_t priority) :
                scheduler_task("UARTtask", 512 * 4, priority)
        {
            // inits
        }
        bool init()
        {
            LPC_SC->PCONP |= (1 << 25); // power for UART3
            LPC_SC->PCLKSEL1 &= ~(3 << 18); // clear bits
            LPC_SC->PCLKSEL1 |= (1 << 18); // set to clk/1

            LPC_PINCON->PINSEL9 &= ~(0xF << 24); // clear both TX and RX (4.28 and 4.29)
            LPC_PINCON->PINSEL9 |= (0xF << 24); // set P4.28 to TX3 and P4.29 to RX3

            LPC_UART3->LCR |= (1 << 7); // set DLAB bit to 1 for configure DLM and DLL

            uint16_t DLMDLL = sys_get_cpu_clock() / ((16 * BAUD_RATE) + 0.5); // CPU=48MHZ, BAUD=38400 = ~78
            LPC_UART3->DLM = (DLMDLL >> 8); // upper 8 bits, should be 0
            LPC_UART3->DLL = (DLMDLL); // lower 8 bits, 78

            LPC_UART3->LCR &= ~(1 << 7); // set DLAB bit to 0 after setting DLM and DLL bits

            LPC_UART3->LCR |= (3 << 0); // 8 bit char length

            LPC_UART3->FCR |= (0x7 << 0); // enable FIFO and reset RX & TX FIFO

            return true;
        }

        bool run(void *p)
        {
            char option = '0';
            char receivedData;
            char message[] = "hello from the..";
            int sizeOfMessage = sizeof(message) / sizeof(char);

            puts("Option 1: Send Data    Option 2: Receive Data");
            scanf("%c", &option);
            if (option == '1')
            {
                puts(
                        "Sending 'hello from the ..' message over to other board...");
                for (int i = 0; i < sizeOfMessage; i++)
                {
                    uart3_putchar(message[i]);
                }
                puts("");
            }
            else
            {
                if (LPC_UART3->LSR & (1 << 0))
                    puts("Receiving message from other board...");
                else
                    puts("No message to receive...");
                while (LPC_UART3->LSR & (1 << 0))
                { // receiver FIFO not empty
                    receivedData = uart3_getchar();
                    printf("%c", receivedData);
                }
                puts("");
            }

            vTaskDelay(1000);
            return true;
        }

        void uart3_putchar(char sendData)
        {
            LPC_UART3->THR = sendData;
            while (!(LPC_UART3->LSR & (1 << 6)))
                ; // if THR reg containing data, inf loop

        }

        char uart3_getchar()
        {
            while (!(LPC_UART3->LSR & (1 << 0)))
                ; // if receiver FIFO is empty (wait to have data), inf loop
            char data = LPC_UART3->RBR;
            return data;
        }
};

class i2cTask: public scheduler_task

{
    public:
        i2cTask(uint8_t priority) :
                scheduler_task("i2c", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            I2C2& i2c2 = I2C2::getInstance();
            i2c2.init_slave(addr, buffer, sizeof(buffer));

            LPC_PINCON->PINSEL2 &= ~(3 << 0); // on board LED9
            LPC_GPIO1->FIODIR |= (1 << 0); //LED output

            LPC_PINCON->PINSEL2 &= ~(3 << 8); // on board LED7
            LPC_GPIO1->FIODIR |= (1 << 4); //LED output

            return true;
        }
        uint8_t addr = 0x31; // slave addr
        uint8_t buffer[256] =
        { 0 };
        uint8_t prev = buffer[5];
        uint8_t prev1 = buffer[9];

        bool run(void *p)
        {
            if (prev != buffer[5])
            { //checking register 0x5 if data is changed
                prev = buffer[5];
                LPC_GPIO1->FIOCLR = (1 << 0); // LED should turn on for 1 second
            }
            else
            {
                LPC_GPIO1->FIOSET = (1 << 0);
            }

            if (prev1 != buffer[9])
            { //checking register 0x9 if data is changed
                prev1 = buffer[9];
                LPC_GPIO1->FIOCLR = (1 << 4); // LED should turn on for 1 second
            }
            else
            {
                LPC_GPIO1->FIOSET = (1 << 4);
            }
            vTaskDelay(1000);
            return true;
        }
};

class Task: public scheduler_task // Skeleton to create a task
{
    public:
        Task(uint8_t priority) :
                scheduler_task("Task", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {

            return true;
        }
        bool run(void *p)
        {
            puts("task1");
            vTaskDelay(1000);
            return true;
        }
};

typedef enum
{
    invalid, lefty, righty, up, down,
} orientation_t;

typedef enum
{
    shared_SensorQueueId,
} sharedHandleId_t;

class sendTask: public scheduler_task // Skeleton to create a task
{
    public:
        sendTask(uint8_t priority) :
                scheduler_task("send", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            QueueHandle_t queue = xQueueCreate(1, sizeof(orientation_t));
            addSharedObject(shared_SensorQueueId, queue);
            return true;
        }
        bool run(void *p)
        {
            int tilt_x = AS.getX();
            int tilt_y = AS.getY();
            int tilt_z = AS.getZ();
            //printf ("x: %i   y: %i   z: %i\n", tilt_x, tilt_y, tilt_z);
            puts("Sending orientation: BEFORE");

            orientation_t orientation;

            if(tilt_x >100 && tilt_y < 200)
            {
                orientation = lefty; //1
            }
            else if(tilt_x < 0 && tilt_y < 100)
            {
                orientation = righty; //2
            }
            else if(tilt_y > 100)
            {
                orientation = up; //3
            }
            else if (tilt_y < 0)
            {
                orientation = down; //4
            }
            else
            {
                orientation = invalid; //0
            }
            if(xQueueSend(getSharedObject(shared_SensorQueueId), &orientation, 1000))
            {
                //printf("sending orientation enum: %i\n", orientation);
                puts("Sending orientation: AFTER");
            }
            else
            {
                puts("FAIL TO SEND\n");
            }
            vTaskDelay(1000);
            return true;
        }
    };

class receiveTask: public scheduler_task // Skeleton to create a task
{
    public:
        receiveTask(uint8_t priority) :
                scheduler_task("receive", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            LPC_PINCON->PINSEL2 &= ~(3 << 0); // on board LED9
            LPC_GPIO1->FIODIR |= (1 << 0); //LED output P1.0

            LPC_PINCON->PINSEL2 &= ~(3 << 16); // on board LED6
            LPC_GPIO1->FIODIR |= (1 << 8); //LED output P1.8

            LPC_PINCON->PINSEL2 &= ~(3 << 2); // on board LED8
            LPC_GPIO1->FIODIR |= (1 << 1); //LED output P1.1

            LPC_PINCON->PINSEL2 &= ~(3 << 8); // on board LED7
            LPC_GPIO1->FIODIR |= (1 << 4); //LED output P1.4

            return true;
        }
        bool run(void *p)
        {
            orientation_t orientation = invalid;
            QueueHandle_t qid = getSharedObject(shared_SensorQueueId);

            if (xQueueReceive(qid, &orientation, portMAX_DELAY))
            {
                //printf("received orientation: %i\n", orientation);
                puts("RECEIVED ORIENTATION!");

                if (orientation == 3)
                {
                    LPC_GPIO1->FIOCLR = (1 << 1); // LED8 turn on
                    LPC_GPIO1->FIOSET = (1 << 4);
                }
                if (orientation == 4)
                {
                    LPC_GPIO1->FIOSET = (1 << 1); // LED7 turn on
                    LPC_GPIO1->FIOCLR = (1 << 4);
                }
                if (orientation == 1)
                {
                    LPC_GPIO1->FIOCLR = (1 << 0); // LED9 turn on
                    LPC_GPIO1->FIOSET = (1 << 8); // LED6
                }
                if (orientation == 2)
                {
                    LPC_GPIO1->FIOCLR = (1 << 8); // LED6
                    LPC_GPIO1->FIOSET = (1 << 0); // LED9
                }
            }
            else
            {
                puts("FAILED TO RECEIVE\n");
            }
            return true;
        }
};

SemaphoreHandle_t signal_t;
SoftTimer timer(500);
void test_ir()
{
    if (timer.expired())
    {
        xSemaphoreGiveFromISR(signal_t, 0);
        puts("hello from interrupt");
        timer.restart();
    }
}

class semaphoreTask: public scheduler_task
{
    public:
        semaphoreTask(uint8_t priority) :
                scheduler_task("semaphore", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            vSemaphoreCreateBinary(signal_t);
            xSemaphoreTake(signal_t, 0);
            return true;
        }

        bool run(void *p)
        {

            if (xSemaphoreTake(signal_t, portMAX_DELAY))
            {
                puts("Semaphore taken");
            }
            vTaskDelay(1000);
            return true;
        }
};


EventGroupHandle_t xEventGroup;


#define bit1    ( 1 << 1 )
#define bit2    ( 1 << 2 )
#define ALL_SYNC_BITS (bit1 | bit2)
#define bit1_stuck (0 << 1)
#define bit2_stuck (0 << 2)


class producer_task: public scheduler_task
{
    public:
        producer_task(uint8_t priority) : //transmit
                scheduler_task("producer", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            QueueHandle_t my_queue = xQueueCreate(10, sizeof(int));
            addSharedObject(shared_SensorQueueId, my_queue);

            return true;
        }

        bool run(void *p)
        {
            int light_value = 0;
            int tx_samples = 0;
            for (int i = 0; i < 100; i++)
            {
                light_value = LS.getRawValue();
                tx_samples += light_value;
                vTaskDelay(1);
            }
            tx_samples = tx_samples / 100;
            if (xQueueSend(getSharedObject(shared_SensorQueueId), &tx_samples,
                    100))
            {
                //printf("sending avg of 100 samples: %i\n", tx_samples);
                xEventGroupSetBits(xEventGroup, // The event group being updated.
                                       bit1);
                           puts("producer\n");
            }
            else
            {
                puts("FAIL TO SEND\n");
            }

            vTaskDelay(1);

            return true;
        }
};

class consumer_task: public scheduler_task
{
    public:
        consumer_task(uint8_t priority) : //receive
                scheduler_task("consumer", 512 * 4, priority)
        {
            /* Nothing to init */

        }

        bool init(void)
        {
            rtc_init();
            return true;
        }

        int array_count = 0;
        int buffer[10] =
        { 0 };
        int rx_samples = 0;

        bool run(void *p)
        {
            QueueHandle_t qid = getSharedObject(shared_SensorQueueId);

            if (xQueueReceive(qid, &rx_samples, portMAX_DELAY))
            {
                buffer[array_count] = rx_samples;
                //printf("avg received:%i   %i   %i\n",array_count,uptime_ms, buffer[array_count]);
                array_count++;
                if (array_count == 10)
                {
                    FILE *sensor = fopen("0:sensor.txt", "a");
                    for (array_count = 0; array_count < 10; array_count++)
                    {
                        fprintf(sensor, "%i seconds  %i\n",
                                rtc_gettime().min * 60 + rtc_gettime().sec,
                                buffer[array_count]);
                    }
                    fclose(sensor);
                    array_count = 0;
                }
                xEventGroupSetBits(xEventGroup, // The event group being updated.
                                    bit2);
                            puts("consumer");
            }
            return true;
        }
};

class watchdog: public scheduler_task
{
    public:
        watchdog(uint8_t priority) : //watch
                scheduler_task("watchdog", 512 * 4, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
            rtc_init();
            xEventGroup = xEventGroupCreate();
            return true;
        }

        bool run(void *p)
        {
            EventBits_t watchdog_bit;
            const TickType_t xTicksToWait = 1000;// / portTICK_PERIOD_MS;

            watchdog_bit = xEventGroupWaitBits(xEventGroup, // The event group being tested.
                    bit1 | bit2, // The bits within the event group to wait for.
                    pdTRUE, // BIT_0 and BIT_4 should be cleared before returning.
                    pdFALSE, // Don't wait for both bits, either bit will do.
                    xTicksToWait); // Wait a maximum of 1s for either bit to be set.

            EventBits_t uxBits;

                    // Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
                    // the event group.  Clear the bits before exiting.
                    uxBits = xEventGroupWaitBits(
                                xEventGroup,    // The event group being tested.
                                bit1 | bit2,  // The bits within the event group to wait for.
                                pdTRUE,         // BIT_0 and BIT_4 should be cleared before returning.
                                pdFALSE,        // Don't wait for both bits, either bit will do.
                                xTicksToWait ); // Wait a maximum of 100ms for either bit to be set.


            //printf("in minutes: %i in seconds: %i\n", rtc_gettime().min, rtc_gettime().sec);

            if (rtc_gettime().min != 0 && rtc_gettime().sec == 0)
            { //every 60 seconds print cpu usage to cpu.txt
                puts("60 seconds has passed\n");
                FILE *output = fopen("0:cpu.txt", "a");
                const char * const taskStatusTbl[] =
                { "RUN", "RDY", "BLK", "SUS", "DEL" };

                // Limit the tasks to avoid heap allocation.
                const unsigned portBASE_TYPE maxTasks = 16;
                TaskStatus_t status[maxTasks];
                uint32_t totalRunTime = 0;
                uint32_t tasksRunTime = 0;
                const unsigned portBASE_TYPE uxArraySize = uxTaskGetSystemState(
                        &status[0], maxTasks, &totalRunTime);

                fprintf(output, "%10s Sta Pr Stack CPU%%          Time\n",
                        "Name");
                for (unsigned priorityNum = 0;
                        priorityNum < configMAX_PRIORITIES; priorityNum++)
                {
                    /* Print in sorted priority order*/
                    for (unsigned i = 0; i < uxArraySize; i++)
                    {
                        TaskStatus_t *e = &status[i];
                        if (e->uxBasePriority == priorityNum)
                        {
                            tasksRunTime += e->ulRunTimeCounter;

                            const uint32_t cpuPercent =
                                    (0 == totalRunTime) ? 0 : e->ulRunTimeCounter
                                                                  / (totalRunTime
                                                                          / 100);
                            const uint32_t timeUs = e->ulRunTimeCounter;
                            const uint32_t stackInBytes = (4
                                    * e->usStackHighWaterMark);

                            fprintf(output, "%10s %s %2u %5u %4u %10u us\n",
                                    e->pcTaskName,
                                    taskStatusTbl[e->eCurrentState],
                                    e->uxBasePriority, stackInBytes, cpuPercent,
                                    timeUs);
                        }
                    }
                }
                fclose(output);
            }



            if ((uxBits & ALL_SYNC_BITS) == ALL_SYNC_BITS)
            { //both set
                FILE *stuck = fopen("0:stuck.txt", "a");
                fprintf(stuck, "%i seconds--- Both bit1 and bit2 are set.\n",
                        rtc_gettime().min * 60 + rtc_gettime().sec);
                fclose(stuck);
                puts("Clear.\n");
            }
            else if ((uxBits & ALL_SYNC_BITS) == (bit2_stuck | bit1_stuck))
            {
                FILE *stuck = fopen("0:stuck.txt", "a");
                fprintf(stuck, "%i seconds--- Both bit1 and bit2 got stuck!\n",
                        rtc_gettime().min * 60 + rtc_gettime().sec);
                fclose(stuck);
                puts("Both bit1 and bit2 got stuck!\n");
            }
            else if ((uxBits & bit1) == 0)
            { //bit1 set
                FILE *stuck = fopen("0:stuck.txt", "a");
                fprintf(stuck, "%i seconds--- Bit1 got stuck!\n",
                        rtc_gettime().min * 60 + rtc_gettime().sec);
                fclose(stuck);
                puts("Bit2 got stuck!\n");
            }
            else if ((uxBits & bit2) == 0)
            { //bit2 set
                FILE *stuck = fopen("0:stuck.txt", "a");
                fprintf(stuck, "%i seconds --- Bit2 got stuck!\n",
                        rtc_gettime().min * 60 + rtc_gettime().sec);
                fclose(stuck);
                puts("Bit1 got stuck!\n");
            }

            puts("watchdog\n");

            //EventBits_t clearbits = xEventGroupClearBits(xEventGroup, ALL_SYNC_BITS);


            vTaskDelay(1000);

            return true;
        }

};

class aCANTask: public scheduler_task // Skeleton to create a task
{
    public:
        aCANTask(uint8_t priority) :
                scheduler_task("CAN", 512 * 4, priority)
        {
            // Nothing to init
        }

        bool init(void)
        {

            uint32_t sclk_khz = 500;
            My_spi1::get_instance()->init(sclk_khz);

            setRunDuration(2000);

            //ssp1_init();

            return true;

        }

        bool run(void *p)
        {

            My_spi1* spi1 = My_spi1::get_instance();
            static Gpio vs_cs(FLASH_PORT, FLASH_PIN, GPIO_OUTPUT, GPIO_HIGH);

            vs_cs.set_state(GPIO_LOW);
            spi1->exchange_byte(0x9F);
            printf("%x\n", spi1->exchange_byte(0x0));
            printf("%x\n", spi1->exchange_byte(0x0));
            printf("%x\n", spi1->exchange_byte(0x0));
            printf("%x\n", spi1->exchange_byte(0x0));
            printf("%x\n", spi1->exchange_byte(0x0));
            vs_cs.set_state(GPIO_HIGH);



            /*
            LPC_GPIO0->FIOCLR = (1 << 6);
            ssp1_exchange_byte(0x9F);
            printf("%x\n", ssp1_exchange_byte(0x0));
            printf("%x\n", ssp1_exchange_byte(0x0));
            printf("%x\n", ssp1_exchange_byte(0x0));
            printf("%x\n", ssp1_exchange_byte(0x0));
            printf("%x\n", ssp1_exchange_byte(0x0));
            LPC_GPIO0->FIOSET = (1 << 6);
            */
            vTaskDelay(1000);
            return true;
        }
};

/*
 // quiz sample #1
QueueHandle_t qh = 0;
void tx1(void *p)
{
    int buffer = 1;
    while (1)
    {
        xQueueSend(qh, &buffer, portMAX_DELAY);
        printf("tx1: sent: %i\n", buffer);
        vTaskSuspend(0); // Suspend this task
    }
}

void tx2(void *p)
{
    int buffer = 2;
    while (1)
    {
        xQueueSend(qh, &buffer, portMAX_DELAY);
        printf("tx2: sent: %i\n", buffer);
        vTaskSuspend(0); // Suspend this task
    }
}

void rx(void *p)
{
    int buffer = 0;
    while (1)
    {
        xQueueReceive(qh, &buffer, portMAX_DELAY);
        printf("RX = %i\n", buffer);
    }
}
*/
/*
 // quiz sample #2
QueueHandle_t qh = 0;

void rx(void *p){
    int item = 0;

    puts("rx task");
    if(xQueueReceive(qh, &item, portMAX_DELAY)){
        puts("Rx received an item!");
    }

    vTaskSuspend(0);
    puts("Rx is suspended!");
}

void tx(void *p){
    int item =0;
    while(1){
        puts("Yield");
        taskYIELD();

        xQueueSend(qh, &item, 0);
        puts("Did I send an item?");

        xQueueSend(qh, &item, portMAX_DELAY);
        puts("I must have sent an item");
    }
}
*/

SemaphoreHandle_t counting_sem;

void consumered(void *p){
    //char *my_task_name = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());

    printf("consumer2: About to get the semaphore...\n");
    xSemaphoreTake(counting_sem,portMAX_DELAY);
    printf("consumer2: I took the semaphore!\n");

    vTaskSuspend(NULL);
}
void consumere(void *p){
    //char *my_task_name = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());

    printf("consumer1: About to get the semaphore...\n");
    xSemaphoreTake(counting_sem,portMAX_DELAY);
    printf("consumer1 I took the semaphore!\n");

    vTaskSuspend(NULL);
}

void producere(void *p){
    puts("About to give the semaphore");
    xSemaphoreGive(counting_sem);
    puts("Given!");

    vTaskSuspend(NULL);
}


int main(void)
{

    //qh = xQueueCreate(3, sizeof(int));
    /*
    // quiz sampel #1
    xTaskCreate(tx1, "tx1", 2000, 0, PRIORITY_LOW, 0);
    xTaskCreate(tx2, "tx2", 2000, 0, PRIORITY_MEDIUM, 0);
    xTaskCreate(rx, "rx", 2000, 0, PRIORITY_HIGH, 0);
    */
    /*
    // quiz sample #2
    xTaskCreate(rx, "rx", 1024, NULL, PRIORITY_HIGH,NULL);
    xTaskCreate(tx, "tx", 1024, NULL, PRIORITY_LOW,NULL);
    */
    /*
    const uint32_t max_count = 3;
    const uint32_t initial_count = 1;
    counting_sem = xSemaphoreCreateCounting(max_count, initial_count);

    xTaskCreate(consumere, "consumer1", 1024, NULL, PRIORITY_HIGH, NULL);
    xTaskCreate(consumered, "consumer2", 1024, NULL, PRIORITY_MEDIUM, NULL);
    xTaskCreate(producere, "producer", 1024, NULL, PRIORITY_LOW, NULL);

     vTaskStartScheduler();
     */
    scheduler_add_task(new CANTask(PRIORITY_MEDIUM));
    //scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
    //scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
    //scheduler_add_task(new watchdog(PRIORITY_HIGH));
    //eint3_enable_port2(5, eint_rising_edge, test); //enable interrupt for Port 2 pins
    //eint3_enable_port2(5, eint_falling_edge, test_ir); //enable interrupt for Port 2 pins
    //scheduler_add_task(new semaphoreTask(PRIORITY_HIGH));
    //scheduler_add_task(new sendTask(PRIORITY_HIGH));
    //scheduler_add_task(new receiveTask(PRIORITY_HIGH));
    //scheduler_add_task(new i2cTask(PRIORITY_MEDIUM));
    //scheduler_add_task(new UARTTask(PRIORITY_MEDIUM));
    //scheduler_add_task(new SPITask(PRIORITY_MEDIUM));
    //scheduler_add_task(new GPIOTask(PRIORITY_MEDIUM));
    //scheduler_add_task(new motorTask(PRIORITY_MEDIUM));
    //scheduler_add_task(new compassI2C(PRIORITY_HIGH));
    //scheduler_add_task(new geoTask(PRIORITY_HIGH));
    //scheduler_add_task(new gpsTask(PRIORITY_HIGH));

    /*
     When both tasks have the same priority, the messages printed are jumbled up because of RTOS's
     preemptive OS tick interrupt. The OS tick interrupt allows same priority task to be time sliced by round robin.
     At startup, since both task have same priority, any task can be executed, but ideally you want the sender to start.
     In an ideal situation, xQueueSend will execute first then run the vTaskDelay for 1second.
     After that, it will wake up receiverTask, where xQueueReceive is executed.
     Actual output:
     x: -27   y: 54   z: 1011
     Sending orientation: BEFORE
     SRECendiEIVEng oD ORrienIENtatiTATIon: ON!
     AFTER

     Ideal output:
     Sending orientation: BEFORE
     Sending orientation enum: AFTER
     RECEIVED ORIENTATION!

     When receiveTask is set to a higher priority, it will first execute the receiverTask, but there is nothing to
     receive so it will be put to sleep. This allows sendTask to run and send the orientation.
     Once it sends it over, it immediately switches/wake up to receiveTask to execute xQueueReceive.
     After executing, it will switch back to sendTask by default of FreeRtos's scheduling algorithm
     Actual Output:
     Sending orientation: BEFORE
     RECEIVED ORIENTATION!
     Sending orientation enum: AFTER

     Questions
     1)What if you use ZERO block time while sending an item to the queue, will that make any difference?
     Block time during a SEND is the maximum time the task will sleep to wait for the queue to have space
     available in the queue. In the case of this lab, it would not make any difference because the receiver
     is woken up every time data is sent to the queue to consume it, which will free up the queue again.
     Generally, it would matter if the queue is full and no consumer to consume the data in the queue to free
     it up.
     2)What is the purpose of the block time during xQueueReceive() ?
     Block time is the maximum time that a task is to be kept sleeping. It will wait for data to be available
     in the queue. When there is data in the queue the block time does not matter as it will move to Ready state.
     Generally, it means that it will sleep for a number of ticks before being woken up automatically to go from
     Block state to Ready state
     */

    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */

    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));
    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
#if 0
    scheduler_add_task(new example_io_demo());
#endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
#if 0
    scheduler_add_task(new example_task());
    scheduler_add_task(new example_alarm());
    scheduler_add_task(new example_logger_qset());
    scheduler_add_task(new example_nv_vars());
#endif

    /**
     * Try the rx / tx tasks together to see how they queue data to each other.
     */
#if 0
    scheduler_add_task(new queue_tx());
    scheduler_add_task(new queue_rx());
#endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
#if 0
    scheduler_add_task(new producer());
    scheduler_add_task(new consumer());
#endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
#if 0
    Uart3 &u3 = Uart3::getInstance();
    u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
    scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
#endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
