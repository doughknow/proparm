#include "RegisterAddresses.h"

void I2CStartRestart()
{
	// See page 493 of ST's RM0383.  We're in master mode, this will start/restart
	// I2C communication.
	ACCESS(I2C1_CR1) |= (1 << 8);
}

void I2CStop()
{
	// See page 493 of ST's RM0383.  We're in master mode, so setting this bit will
	// "stop generation after the current byte transfer or after the current Start
	// condition is sent".
	ACCESS(I2C1_CR1) |= (1 << 9);
}

void I2CEnableAcknowledge()
{
	// See page 493 of ST's RM0383.  Setting this bit will "acknowledge returned
	// after a byte is received".  The accelerometer will acknowledge when a byte
	// is received.
	ACCESS(I2C1_CR1) |= (1 << 10);
}

void I2CDisableAcknowledge()
{
	// See page 493 of ST's RM0383.  We turn acknowledge off here.
	ACCESS(I2C1_CR1) &= ~(1 << 10);
}

void I2CSendSlaveAddress(unsigned short address)
{
	// See page 500.  We wait for the start bit to be generated.
	while((ACCESS(I2C1_SR1) & 1) == 0);

	// Write the address into the I2C data register.
	ACCESS(I2C1_DR) = address;

	// See page 500.  Wait until the end of transmission.
	while((ACCESS(I2C1_SR1) & (1 << 1)) == 0);

	// See page 500 of the datasheet, under bit 1 ADDR: "This bit is cleared by software
	// reading SR1 register followed reading SR2"
	ACCESS(I2C1_SR1);
	ACCESS(I2C1_SR2);
}

void I2CSendRegister(unsigned short registerAddress)
{
	// See page 499. We wait until the Tx data register is empty.
	while((ACCESS(I2C1_SR1) & (1 << 7)) == 0);

	// Put the register address into the data register
	ACCESS(I2C1_DR) = registerAddress;

	// Wait until the transfer is complete.
	while((ACCESS(I2C1_SR1) & (1 << 7)) == 0);
}

void I2CWaitIfBusy()
{
	// See page 502 of the datasheet.  Bit 1 of SR2 will be set when the I2C bus is busy.
	while((ACCESS(I2C1_SR2) & (1 << 1)) == (1 << 1));
}

void I2CWriteByte(unsigned char data)
{
	// Write the data to the data register
	ACCESS(I2C1_DR) = data;

	// See page 500 of the datasheet.  Bit 2 will be set when the data transfer has succeeded.
	while((ACCESS(I2C1_SR1) & (1 << 2)) == 0);
}

unsigned char I2CGetData()
{
	// See page 499 of the datasheet.  Bit 6 of SR1 will be set when receiver data exist the
	// data register.
	while((ACCESS(I2C1_SR1) & (1 << 6)) == 0) { }

	// Return the data
	return ACCESS(I2C1_DR);
}
