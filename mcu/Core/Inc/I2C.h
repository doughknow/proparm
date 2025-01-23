#pragma once

void I2CStartRestart();
void I2CStop();
void I2CEnableAcknowledge();
void I2CDisableAcknowledge();
void I2CSendSlaveAddress(unsigned short address);
void I2CSendRegister(unsigned short registerAddress);
void I2CWaitIfBusy();
void I2CWriteByte(unsigned char data);
unsigned char I2CGetData();