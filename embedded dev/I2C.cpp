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


int main(void)
{

    scheduler_add_task(new i2cTask(PRIORITY_MEDIUM));
    

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
