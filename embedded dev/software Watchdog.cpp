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


int main(void)
{
 
    scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
    scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
    scheduler_add_task(new watchdog(PRIORITY_HIGH));

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
