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

#include <string.h>         // memcpy
#include <stdio.h>
#include "i2c_base.hpp"
#include "lpc_sys.h"
#include <iostream>
#include <printf_lib.h>

using namespace std;

/**
 * Instead of using a dedicated variable for read vs. write, we just use the LSB of
 * the user address to indicate read or write mode.
 */
#define I2C_SET_READ_MODE(addr)     (addr |= 1)     ///< Set the LSB to indicate read-mode
#define I2C_SET_WRITE_MODE(addr)    (addr &= 0xFE)  ///< Reset the LSB to indicate write-mode
#define I2C_READ_MODE(addr)         (addr & 1)      ///< Read address is ODD
#define I2C_WRITE_ADDR(addr)        (addr & 0xFE)   ///< Write address is EVEN
#define I2C_READ_ADDR(addr)         (addr | 1)      ///< Read address is ODD



void I2C_Base::handleInterrupt()
{
    /* If transfer finished (not busy), then give the signal */
    if (busy != i2cStateMachine()) {
        long higherPriorityTaskWaiting = 0;
        xSemaphoreGiveFromISR(mTransferCompleteSignal, &higherPriorityTaskWaiting);
        portEND_SWITCHING_ISR(higherPriorityTaskWaiting);
    }
}

uint8_t I2C_Base::readReg(uint8_t deviceAddress, uint8_t registerAddress)
{
    uint8_t byte = 0;
    readRegisters(deviceAddress, registerAddress, &byte, 1);
    return byte;
}

bool I2C_Base::readRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToRead)
{
    I2C_SET_READ_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToRead);
}

bool I2C_Base::writeReg(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value)
{
    return writeRegisters(deviceAddress, registerAddress, &value, 1);
}

bool I2C_Base::writeRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToWrite)
{
    I2C_SET_WRITE_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToWrite);
}

bool I2C_Base::transfer(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t transferSize)
{
    bool status = false;
    if(mDisableOperation || !pData) {
        return status;
    }

    // If scheduler not running, perform polling transaction
    if(taskSCHEDULER_RUNNING != xTaskGetSchedulerState())
    {
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish
        const uint64_t timeout = sys_get_uptime_ms() + I2C_TIMEOUT_MS;
        while (!xSemaphoreTake(mTransferCompleteSignal, 0)) {
            if (sys_get_uptime_ms() > timeout) {
                break;
            }
        }

        status = (0 == mTransaction.error);
    }
    else if (xSemaphoreTake(mI2CMutex, OS_MS(I2C_TIMEOUT_MS)))
    {
        // Clear potential stale signal and start the transfer
        xSemaphoreTake(mTransferCompleteSignal, 0);
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish and copy the data if it was read mode
        if (xSemaphoreTake(mTransferCompleteSignal, OS_MS(I2C_TIMEOUT_MS))) {
            status = (0 == mTransaction.error);
        }

        xSemaphoreGive(mI2CMutex);
    }

    return status;
}

bool I2C_Base::checkDeviceResponse(uint8_t deviceAddress)
{
    uint8_t dummyReg = 0;
    uint8_t notUsed = 0;

    // The I2C State machine will not continue after 1st state when length is set to 0
    uint32_t lenZeroToTestDeviceReady = 0;

    return readRegisters(deviceAddress, dummyReg, &notUsed, lenZeroToTestDeviceReady);
}

I2C_Base::I2C_Base(LPC_I2C_TypeDef* pI2CBaseAddr) :
        mpI2CRegs(pI2CBaseAddr),
        mDisableOperation(false)
{
    mI2CMutex = xSemaphoreCreateMutex();
    mTransferCompleteSignal = xSemaphoreCreateBinary();

    /// Binary semaphore needs to be taken after creating it
    xSemaphoreTake(mTransferCompleteSignal, 0);

    if((unsigned int)mpI2CRegs == LPC_I2C0_BASE)
    {
        mIRQ = I2C0_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C1_BASE)
    {
        mIRQ = I2C1_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C2_BASE)
    {
        mIRQ = I2C2_IRQn;
    }
    else {
        mIRQ = (IRQn_Type)99; // Using invalid IRQ on purpose
    }
}

bool I2C_Base::init(uint32_t pclk, uint32_t busRateInKhz)
{
    // Power on I2C
    switch(mIRQ) {
        case I2C0_IRQn: lpc_pconp(pconp_i2c0, true);  break;
        case I2C1_IRQn: lpc_pconp(pconp_i2c1, true);  break;
        case I2C2_IRQn: lpc_pconp(pconp_i2c2, true);  break;
        default: return false;
    }

    mpI2CRegs->I2CONCLR = 0x6C;           // Clear ALL I2C Flags

    /**
     * Per I2C high speed mode:
     * HS mode master devices generate a serial clock signal with a HIGH to LOW ratio of 1 to 2.
     * So to be able to optimize speed, we use different duty cycle for high/low
     *
     * Compute the I2C clock dividers.
     * The LOW period can be longer than the HIGH period because the rise time
     * of SDA/SCL is an RC curve, whereas the fall time is a sharper curve.
     */
    const uint32_t percent_high = 40;
    const uint32_t percent_low = (100 - percent_high);
    const uint32_t freq_hz = (busRateInKhz > 1000) ? (100 * 1000) : (busRateInKhz * 1000);
    const uint32_t half_clock_divider = (pclk / freq_hz) / 2;
    mpI2CRegs->I2SCLH = (half_clock_divider * percent_high) / 100;
    mpI2CRegs->I2SCLL = (half_clock_divider * percent_low ) / 100;

    // Set I2C slave address and enable I2C
    mpI2CRegs->I2ADR0 = 0;
    mpI2CRegs->I2ADR1 = 0;
    mpI2CRegs->I2ADR2 = 0;
    mpI2CRegs->I2ADR3 = 0;

    // Enable I2C and the interrupt for it,
    mpI2CRegs->I2CONSET = 0x40;
    NVIC_EnableIRQ(mIRQ);

    return true;
}

void I2C_Base::init_slave(uint8_t slave_addr, uint8_t* buffer, int size){
    mpI2CRegs->I2ADR0 = slave_addr; //write=0, read= 1, lsb
    mpI2CRegs->I2CONCLR = 0x6C;
    mpI2CRegs->I2CONSET = 0x44; //I2EN (enable I2C function) & AA (ack its slave addr) enabled, Table 383
    slave_trans.pSlaveData = buffer;
    slave_trans.regAddrFirst0x80 = true;
    slave_trans.trxSize = size;

}

/// Private ///

void I2C_Base::i2cKickOffTransfer(uint8_t devAddr, uint8_t regStart, uint8_t* pBytes, uint32_t len)
{
    mTransaction.error     = 0;
    mTransaction.slaveAddr = devAddr;
    mTransaction.firstReg  = regStart;
    mTransaction.trxSize   = len;
    mTransaction.pMasterData   = pBytes;

    // Send START, I2C State Machine will finish the rest., master transmitter mode table 382
    mpI2CRegs->I2CONSET = 0x20;
}

/*
 * I2CONSET bits
 * 0x04 AA
 * 0x08 SI
 * 0x10 STOP
 * 0x20 START
 * 0x40 ENABLE
 *
 * I2CONCLR bits
 * 0x04 AA
 * 0x08 SI
 * 0x20 START
 * 0x40 ENABLE
 */
I2C_Base::mStateMachineStatus_t I2C_Base::i2cStateMachine()
{
    enum {
        // General states :
        busError        = 0x00,
        start           = 0x08,
        repeatStart     = 0x10,
        arbitrationLost = 0x38,

        // Master Transmitter States:
        slaveAddressAcked  = 0x18,
        slaveAddressNacked = 0x20,
        dataAckedBySlave   = 0x28,
        dataNackedBySlave  = 0x30,

        // Master Receiver States:
        readAckedBySlave      = 0x40,
        readModeNackedBySlave = 0x48,
        dataAvailableAckSent  = 0x50,
        dataAvailableNackSent = 0x58,

        // Slave Receiver States
        receivedOwnAddrAckReturnReceive = 0x60,
        dataReceivedAckReturn = 0x80,
        stopCond = 0xA0,
        arblost = 0x68,

        // Slave Transmitter States
        receivedOwnAddrAckReturnTransmit = 0xA8,
        dataTransmittedAckReceived = 0xB8,
        dataTransmittedNACKreceived= 0xC0,
        lastDataTransmittedAckReceived= 0xC8
    };

    mStateMachineStatus_t state = busy;

    /*
     ***********************************************************************************************************
     * Write-mode state transition :
     * start --> slaveAddressAcked --> dataAckedBySlave --> ... (dataAckedBySlave) --> (stop)
     *
     * Read-mode state transition :
     * start --> slaveAddressAcked --> dataAcked --> repeatStart --> readAckedBySlave
     *  For 2+ bytes:  dataAvailableAckSent --> ... (dataAvailableAckSent) --> dataAvailableNackSent --> (stop)
     *  For 1  byte :  dataAvailableNackSent --> (stop)
     ***********************************************************************************************************
     */

    /* Me being lazy and using #defines instead of inline functions :( */
    #define clearSIFlag()       mpI2CRegs->I2CONCLR = (1<<3)
    #define setSTARTFlag()      mpI2CRegs->I2CONSET = (1<<5)
    #define clearSTARTFlag()    mpI2CRegs->I2CONCLR = (1<<5)
    #define setAckFlag()        mpI2CRegs->I2CONSET = (1<<2)
    #define setNackFlag()       mpI2CRegs->I2CONCLR = (1<<2)

    /* yep ... lazy again */
    #define setStop()           clearSTARTFlag();                           \
                                mpI2CRegs->I2CONSET = (1<<4);               \
                                clearSIFlag();                              \
                                while((mpI2CRegs->I2CONSET&(1<<4)));        \
                                if(I2C_READ_MODE(mTransaction.slaveAddr))   \
                                    state = readComplete;                   \
                                else                                        \
                                    state = writeComplete;

    //cout << "NS: " << hex << mpI2CRegs->I2STAT << "\n";
    ////cout << "Data: " << mpI2CRegs->I2DAT << "\n";
    //u0_dbg_printf("I2C HW state: %u 0x%02X\n", mpI2CRegs->I2STAT, mpI2CRegs->I2STAT);
    switch (mpI2CRegs->I2STAT)
    {
        case start: //0x08
            mpI2CRegs->I2DAT = I2C_WRITE_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;
        case repeatStart: //0x10
            mpI2CRegs->I2DAT = I2C_READ_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;

        case slaveAddressAcked: //0x18
            clearSTARTFlag();
            // No data to transfer, this is used just to test if the slave responds
            if(0 == mTransaction.trxSize) {
                setStop();
            }
            else {
                mpI2CRegs->I2DAT = mTransaction.firstReg;
                clearSIFlag();
            }
            break;

        case dataAckedBySlave: //0x28
            if (I2C_READ_MODE(mTransaction.slaveAddr)) {
                setSTARTFlag(); // Send Repeat-start for read-mode
                clearSIFlag();
            }
            else {
                if(0 == mTransaction.trxSize) {
                    setStop();
                }
                else {
                    mpI2CRegs->I2DAT = *(mTransaction.pMasterData);
                    ++mTransaction.pMasterData;
                    --mTransaction.trxSize;
                    clearSIFlag();

                }
            }
            break;

        /* In this state, we are about to initiate the transfer of data from slave to us
         * so we are just setting the ACK or NACK that we'll do AFTER the byte is received.
         */
        case readAckedBySlave: //0x40
            clearSTARTFlag();
            if(mTransaction.trxSize > 1) {
                setAckFlag();  // 1+ bytes: Send ACK to receive a byte and transition to dataAvailableAckSent
            }
            else {
                setNackFlag();  //  1 byte : NACK next byte to go to dataAvailableNackSent for 1-byte read.
            }

            clearSIFlag();

            break;

        case dataAvailableAckSent: //0x50
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            ++mTransaction.pMasterData;
            --mTransaction.trxSize;

            if(1 == mTransaction.trxSize) { // Only 1 more byte remaining
                setNackFlag();// NACK next byte --> Next state: dataAvailableNackSent
            }
            else {
                setAckFlag(); // ACK next byte --> Next state: dataAvailableAckSent(back to this state)
            }

            clearSIFlag();

            break;

        case dataAvailableNackSent: // Read last-byte from Slave  //0x58
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            setStop();

            break;

        case arbitrationLost: //0x38
            // We should not issue stop() in this condition, but we still need to end our  transaction.
            state = I2C_READ_MODE(mTransaction.slaveAddr) ? readComplete : writeComplete;
            mTransaction.error = mpI2CRegs->I2STAT;
            break;

        case receivedOwnAddrAckReturnReceive: // 0x60
            slave_trans.regAddrFirst0x80= true;
            setAckFlag();
            clearSIFlag();

            break;

        case dataReceivedAckReturn: //0x80
            if (slave_trans.regAddrFirst0x80){ //during slave receive, will enter here twice, thus need to store data
                 if(mpI2CRegs->I2DAT < slave_trans.trxSize){ //into mapped register, slave transmit only go here once
                     slave_trans.regNum = mpI2CRegs->I2DAT; // First 0x80 is the register address
                 }else{
                     slave_trans.regNum = 0;
                 }
                 slave_trans.regAddrFirst0x80 = false; // 2nd time during receive, should not enter here and put data
                                                         // in register
                }
            else{
                slave_trans.pSlaveData[slave_trans.regNum] = mpI2CRegs->I2DAT;
                // put data into the chosen register
            }
            setAckFlag();
            clearSIFlag();
            break;

        case stopCond: //0xA0
            setAckFlag();
            clearSIFlag();
            break;

        case receivedOwnAddrAckReturnTransmit: // 0xA8
            mpI2CRegs->I2DAT = slave_trans.pSlaveData[slave_trans.regNum++];  // read from register
            if (slave_trans.regNum == slave_trans.trxSize){
                slave_trans.regNum = 0;
            }
            setAckFlag();
            clearSIFlag();
            break;

        case dataTransmittedAckReceived: //0xB8 , back to back reads
            mpI2CRegs->I2DAT = slave_trans.pSlaveData[slave_trans.regNum++]; // back to back reads, increment to go to
            if (slave_trans.regNum == slave_trans.trxSize){                    // next register
                slave_trans.regNum = 0;
            }
            setAckFlag();
            clearSIFlag();

            break;

        case dataTransmittedNACKreceived: // 0xC0,  end of slave transmit
            setAckFlag();
            clearSIFlag();
            break;

        case arblost:
        case slaveAddressNacked:    // no break  0x20
        case dataNackedBySlave:     // no break  0x30
        case readModeNackedBySlave:// no break  0x48
        case busError:              // no break  0x00
        default:
            mTransaction.error = mpI2CRegs->I2STAT;
            setStop();
            break;
    }

    return state;
}
