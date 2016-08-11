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


int main(void)
{  
    scheduler_add_task(new SPITask(PRIORITY_MEDIUM));

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
