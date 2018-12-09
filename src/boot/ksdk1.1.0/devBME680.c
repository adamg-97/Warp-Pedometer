/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceBME680State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



void
initBME680(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskPressure | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
writeSensorRegisterBME680(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	enableI2Cpins(menuI2cPullupValue);

	/*
	 *	Wait for supply and pull-ups to settle.
	 */
	OSA_TimeDelay(100);

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							200);
	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorBME680(uint8_t payloadCtrl_Meas, uint8_t payloadCtrl_Hum, uint8_t payloadConfig, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;

	i2cWriteStatus = writeSensorRegisterBME680(kWarpSensorBME680Ctrl_Hum /* register address Ctrl_Hum */,
							payloadCtrl_Hum /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBME680(kWarpSensorBME680Ctrl_Meas /* register address Ctrl_Meas */,
							payloadCtrl_Meas /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBME680(kWarpSensorBME680Config /* register address Config */,
							payloadConfig /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBME680(kWarpSensorBME680Ctrl_Meas /* register address Ctrl_Meas */,
							payloadCtrl_Meas + 0x01/* payload to forced mode */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Write Error, error %d", i2cWriteStatus);
	}
}

WarpStatus
readSensorRegisterBME680(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	switch (deviceRegister)
	{
		case 0x73: case 0xE0: case 0xD0: case 0x75: case 0x74: case 0x72:
		case 0x71: case 0x70: case 0x64: case 0x65: case 0x66: case 0x67:
		case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D:
		case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E: case 0x5F:
		case 0x60: case 0x61: case 0x62: case 0x63: case 0x50: case 0x51:
		case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57:
		case 0x58: case 0x59: case 0x2B: case 0x2A: case 0x26: case 0x25:
		case 0x24: case 0x23: case 0x22: case 0x21: case 0x20: case 0x1F:
		case 0x1D:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBME680State.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (retrieve measurement)\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceBME680State.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataBME680(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint8_t readSensorRegisterValueXLSB;
	uint32_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680press_msb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680press_lsb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680press_xlsb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
	readSensorRegisterValueCombined =
			((readSensorRegisterValueMSB & 0xFF)<<12) +
			((readSensorRegisterValueLSB & 0xFF)<<4) +
			((readSensorRegisterValueXLSB & 0xF0)>>4);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680temp_msb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680temp_lsb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680temp_xlsb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
	readSensorRegisterValueCombined =
			((readSensorRegisterValueMSB & 0xFF)<<12) +
			((readSensorRegisterValueLSB & 0xFF)<<4) +
			((readSensorRegisterValueXLSB & 0xF0)>>4);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680hum_msb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBME680(kWarpSensorBME680hum_lsb);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BME680 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];

	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
}