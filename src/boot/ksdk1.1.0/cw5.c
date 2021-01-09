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

#include "cw5.h"
#include "devMMA8451Q.h"
#include "devSSD1331.h"



extern volatile WarpI2CDeviceState	deviceMMA8451QState;



void mainScript() {

	//set up screen and fill green
	devSSD1331init();

	SEGGER_RTT_printf(0, "\n\nSTART\n\n");

	uint32_t data = readAccel();

	SEGGER_RTT_printf(0, "\n\nFINISH\n\n");

	//fill screen red
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(0x00); //Column Start
	writeCommand(0x00); //Row Start
	writeCommand(0x5F); //Column End
	writeCommand(0x3f); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0x00); //Line Colour green
	writeCommand(0x00); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0x00); //Fill Colour green
	writeCommand(0x00); //Fill Colour blue

	OSA_TimeDelay(500);

	//clear screen
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);

	displayData(data);
}

uint32_t readAccel() {

	uint16_t	readSensorRegisterValueLSB;
	uint16_t        readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	uint16_t        i2cPullUpValue = 32768;
	int		numberOfReadings = 44;
	int		delay = 30; //milliseconds

	uint32_t	startTime;

	int16_t		xdata[numberOfReadings];

	enableI2Cpins(i2cPullUpValue);

	configureSensorMMA8451Q(0x00, 0x01, i2cPullUpValue);
	writeSensorRegisterMMA8451Q(0x2A, 0x00, i2cPullUpValue); //set standby mode
	writeSensorRegisterMMA8451Q(0x0E, 0x02, i2cPullUpValue); //set 4g scale - can only do this in standby modeoe
	writeSensorRegisterMMA8451Q(0x2A, 0x01, i2cPullUpValue); //set actve mode

	for (int i=0; i<numberOfReadings; i++) {
		
		startTime = OSA_TimeGetMsec();

		i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2);
		if (i2cReadStatus == kWarpStatusOK) {
	
			readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
			readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
			readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
			readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
			
			xdata[i] = (int) (readSensorRegisterValueCombined/100 + 0.5);
		}	
		else {
//			SEGGER_RTT_printf(0, "\nError reading register");
		}


		while((OSA_TimeGetMsec() - startTime) < delay) {}
	}

	disableI2Cpins();
	
	int16_t		maxAccel = 0;
	int		maxAccelIndex;
	int16_t 	velocity[numberOfReadings-1];
	int16_t 	distance[numberOfReadings-2];

	float		intermediateForCalcs;

	for (int i=0; i<numberOfReadings; i++) {

		SEGGER_RTT_printf(0, "\na: %d", xdata[i]);

		//get maximum acceleration
		if (abs(xdata[i]) > maxAccel) {
			maxAccel = abs(xdata[i]);
			maxAccelIndex = i;
		}

		if (i < numberOfReadings-1) {
			intermediateForCalcs = (float) (xdata[i] + xdata[i+1]) * (float) delay/20;
			velocity[i] = (int) intermediateForCalcs;
			if (i > 0) {
				velocity[i] += velocity[i-1];

				intermediateForCalcs = (float) ((velocity[i-1] + velocity[i]) * delay) /2000;

				distance[i-1] = (int) intermediateForCalcs;
				if (i > 1) {
					distance[i-1] += distance[i-2];
				}
				SEGGER_RTT_printf(0, "\nd: %d", distance[i-1]);		
			}
			SEGGER_RTT_printf(0, "\nv: %d", velocity[i]);
		}
	}

        
	if (maxAccel > 99) {maxAccel = 99;}
	uint32_t returnVal = maxAccel;


	if (maxAccelIndex == numberOfReadings-1) {
		maxAccelIndex -= 1;
	}


//	SEGGER_RTT_printf(0, "Velocity at impact is %dcm/s\n\n", velocity[maxAccelIndex]);
	if (abs(velocity[maxAccelIndex]) > 99) {velocity[maxAccelIndex] = 99;}
	returnVal += abs(velocity[maxAccelIndex]) * 100;


	if (maxAccelIndex == numberOfReadings-2) {
		maxAccelIndex -= 1;			       
	}


//	SEGGER_RTT_printf(0, "Distance before impact is %dcm", distance[maxAccelIndex]);
	if (abs(distance[maxAccelIndex]) > 99) {distance[maxAccelIndex] = 99;}
	returnVal += abs(distance[maxAccelIndex]) * 10000;


//	SEGGER_RTT_printf(0, "Distance after impact is %dcm\n", distance[numberOfReadings-3]-distance[maxAccelIndex]);
	if (abs(distance[numberOfReadings-3]-distance[maxAccelIndex]) > 99) {distance[numberOfReadings-3] = 99+distance[maxAccelIndex];}
	returnVal += abs(distance[numberOfReadings-3]-distance[maxAccelIndex]) * 1000000 + 100000000;

	return returnVal;
}

void displayData (uint32_t n) {

	uint32_t 	dataToDisplay;

	while(true) {

		dataToDisplay = n;

		while (dataToDisplay > 1) { 
			while (GPIO_DRV_ReadPinInput(kWarpPinSW3) == 1) {}
			writeNumber(dataToDisplay % 100);
			OSA_TimeDelay(500);

			dataToDisplay /= 100;
		}
	}
}


void writeNumber(int n) {

	writeCommand(kSSD1331CommandCLEAR);

	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);

	int units = n % 10;

	int tens = n/10 % 10;

	switch(tens) {
		case 0:
			drawA(true);
			drawB(true);
			drawC(true);
			drawD(true);
			drawE(true);
			drawF(true);
			break;
		case 1:
			drawB(true);
			drawC(true);
			break;
		case 2:
			drawA(true);
			drawB(true);
			drawG(true);
			drawE(true);
			drawD(true);
			break;
		case 3:
			drawA(true);
			drawB(true);
			drawG(true);
			drawC(true);
			drawD(true);
			break;
		case 4:
			drawF(true);
			drawG(true);
			drawB(true);
			drawC(true);
			break;
		case 5:
			drawA(true);
			drawF(true);
			drawG(true);
			drawC(true);
			drawD(true);
			break;
		case 6:
			drawA(true);
			drawF(true);
			drawE(true);
			drawD(true);
			drawC(true);
			drawG(true);
			break;
		case 7:
			drawA(true);
			drawB(true);
			drawC(true);
			break;
		case 8:
			drawA(true);
			drawF(true);
			drawG(true);
			drawC(true);
			drawD(true);
			drawE(true);
			drawB(true);
			break;
		case 9:
			drawA(true);
			drawF(true);
			drawG(true);
			drawB(true);
			drawC(true);
			break;
		default:
			break;
	}

	switch (units) {
		case 0:
			drawA(false);
	    		drawB(false);
			drawC(false);
			drawD(false);
			drawE(false);
			drawF(false);
			break;
		case 1:
			drawB(false);
			drawC(false);
			break;
		case 2:
			drawA(false);
			drawB(false);
			drawG(false);
			drawE(false);
			drawD(false);
			break;
		case 3:
			drawA(false);
			drawB(false);
			drawG(false);
			drawC(false);
			drawD(false);
			break;
		case 4:
			drawF(false);
			drawG(false);
			drawB(false);
			drawC(false);
			break;
		case 5:
			drawA(false);
			drawF(false);
			drawG(false);
			drawC(false);
			drawD(false);
			break;
		case 6:
			drawA(false);
			drawF(false);
			drawE(false);
			drawD(false);
			drawC(false);
			drawG(false);
			break;
		case 7:
			drawA(false);
			drawB(false);
			drawC(false);
			break;
		case 8:
			drawA(false);
			drawF(false);
			drawG(false);
			drawC(false);
			drawD(false);
			drawE(false);
			drawB(false);
			break;
		case 9:
			drawA(false);
			drawF(false);
			drawG(false);
			drawB(false);
			drawC(false);
			break;
		default:
			break;
	}
}

void drawA(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x08+pixels_shift); //Column Start
	writeCommand(0x07); //Row Start
	writeCommand(0x2B+pixels_shift); //Column End
	writeCommand(0x0C); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue

}

void drawB(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x27+pixels_shift); //Column Start
	writeCommand(0x07); //Row Start
	writeCommand(0x2B+pixels_shift); //Column End
	writeCommand(0x22); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

void drawC(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x27+pixels_shift); //Column Start
	writeCommand(0x1D); //Row Start
	writeCommand(0x2B+pixels_shift); //Column End
	writeCommand(0x38); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

void drawD(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x08+pixels_shift); //Column Start
	writeCommand(0x33); //Row Start
	writeCommand(0x2B+pixels_shift); //Column End
	writeCommand(0x38); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

void drawE(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x08+pixels_shift); //Column Start
	writeCommand(0x1D); //Row Start
	writeCommand(0x0C+pixels_shift); //Column End
	writeCommand(0x38); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

void drawF(bool tens) {

	int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;
	}

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x08+pixels_shift); //Column Start
	writeCommand(0x07); //Row Start
	writeCommand(0x0C+pixels_shift); //Column End
	writeCommand(0x22); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

void drawG(bool tens) {

        int pixels_shift;

	if (tens) {
		pixels_shift = 0x00;
	} else {
		pixels_shift = 0x2C;										        }

	writeCommand(kSSD1331CommandDRAWRECT);

	writeCommand(0x08+pixels_shift); //Column Start
	writeCommand(0x1D); //Row Start
	writeCommand(0x2B+pixels_shift); //Column End
	writeCommand(0x22); //Row End
	writeCommand(0xFF); //Line Colour red
	writeCommand(0xFF); //Line Colour green
	writeCommand(0xFF); //Line Colour blue
	writeCommand(0xFF); //Fill Colour red
	writeCommand(0xFF); //Fill Colour green
	writeCommand(0xFF); //Fill Colour blue
}

