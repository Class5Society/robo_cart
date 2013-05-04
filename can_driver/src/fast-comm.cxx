//*****************************************************************************
//
// bdc-comm.cxx - The main control loop for the bdc-comm application.
//
// Copyright (c) 2009-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9107 of the Stellaris Firmware Development Package.
//
//*****************************************************************************

#include "can_driver/can_driver.h"

//*****************************************************************************
//
// This function is used to set the currently active device ID that is being
// communicated with.
//
//*****************************************************************************
int fastCmdID(uint32_t jagId) {

	//
	// Set the ID
	//
	g_ulID = jagID;

	return (0);
}

//*****************************************************************************
//
// This command is used to toggle if the heart beat messages are being send
// out.
//
//*****************************************************************************
int fastCmdHeartbeat() {
	//
	// Just toggle the heart beat mode.
	//
	g_ulHeartbeat ^= 1;

	return (0);
}


//*****************************************************************************
//
// This command controls the setting when running in Position control mode.
//
//*****************************************************************************
int fastCmdPosEnable(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Enable Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_EN | g_ulID, (unsigned char *) plValue, 4);

	WaitForAck(LM_API_ACK | g_ulID, 10);

}

int fastCmdPosDis(void)
{
	//
	// Disable Position control mode.
	//
	UARTSendMessage(LM_API_POS_DIS | g_ulID, 0, 0);

	WaitForAck(LM_API_ACK | g_ulID, 10);

}

int fastCmdPosSet(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Set Position.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_SET | g_ulID, (unsigned char *) plValue, 4);
	WaitForAck(LM_API_ACK | g_ulID, 10);

}


int fastCmdPosP(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// P value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_PC | g_ulID, (unsigned char *) plValue, 4);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

int fastCmdPosI(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Set the I value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_IC | g_ulID, (unsigned char *) plValue, 4);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

int fastCmdPosD(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Set the D value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_DC | g_ulID, (unsigned char *) plValue, 4);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

int fastCmdPosRef(int32_t inpValue)
{
	int32_t plValue[2];

        plValue[0] = inpValue;
	//
	// Limit the value to valid values.
	//
	if (plValue[0] < 0) {
             plValue[0] = 0;
	}
	if (plValue[0] > 255) {
		plValue[0] = 255;
	}

	//
	// Set the reference in Position control mode.
	//
	UARTSendMessage(LM_API_POS_REF | g_ulID, (unsigned char *) plValue, 1);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

int fastCmdPosSetNoAck(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Set Position.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_SET_NO_ACK | g_ulID, (unsigned char *) plValue, 4);

}


int CmdPosition(int argc, char *argv[]) {
	int32_t plValue[2], lTemp;
	unsigned short *pusData;
	unsigned char *pucData;
	int iIdx;

	pusData = (unsigned short *) plValue;
	pucData = (unsigned char *) plValue;


//*****************************************************************************
//
// This command handles status requests for devices.
//
//*****************************************************************************
int CmdStatus(int argc, char *argv[]) {
	unsigned char ucData;

	if ((argc > 1) && (strcmp(argv[1], "vout") == 0)) {
		//
		// Request the current Voltage output setting.
		//
		UARTSendMessage(LM_API_STATUS_VOLTOUT | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "vbus") == 0)) {
		//
		// Request the current Vbus value.
		//
		UARTSendMessage(LM_API_STATUS_VOLTBUS | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "fault") == 0)) {
		if (argc > 2) {
			ucData = 1;

			//
			// Request a clear of the current Fault value.
			//
			UARTSendMessage(LM_API_STATUS_FAULT | g_ulID, &ucData, 1);
		} else {
			//
			// Request the current Fault value.
			//
			UARTSendMessage(LM_API_STATUS_FAULT | g_ulID, 0, 0);
		}

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "cur") == 0)) {
		//
		// Request the current Current value.
		//
		UARTSendMessage(LM_API_STATUS_CURRENT | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "temp") == 0)) {
		//
		// Request the current temperature.
		//
		UARTSendMessage(LM_API_STATUS_TEMP | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "pos") == 0)) {
		//
		// Request the current position.
		//
		UARTSendMessage(LM_API_STATUS_POS | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "speed") == 0)) {
		//
		// Request the current speed.
		//
		UARTSendMessage(LM_API_STATUS_SPD | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "limit") == 0)) {
		//
		// Request the limit switch settings.
		//
		UARTSendMessage(LM_API_STATUS_LIMIT | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "power") == 0)) {
		//
		// If there is a second argument then this is a set request.
		//
		if (argc > 2) {
			ucData = 1;
			UARTSendMessage(LM_API_STATUS_POWER | g_ulID, &ucData, 1);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// A command with no data is a get request.
			//
			UARTSendMessage(LM_API_STATUS_POWER | g_ulID, 0, 0);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		}
	}

	else if ((argc > 1) && (strcmp(argv[1], "cmode") == 0)) {
		//
		// Get the current control mode.
		//
		UARTSendMessage(LM_API_STATUS_CMODE | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "vout2") == 0)) {
		//
		// Request the current voltage output setting.
		//
		UARTSendMessage(LM_API_STATUS_VOUT | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "stkyfault") == 0)) {
		//
		// Request the current Sticky Fault value.
		//
		UARTSendMessage(LM_API_STATUS_STKY_FLT | g_ulID, 0, 0);

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else if ((argc > 1) && (strcmp(argv[1], "faultcnts") == 0)) {
		if (argc > 2) {
			//
			// Convert the string to unsigned char value.
			//
			ucData = (char) strtol(argv[2], 0, 0);

			//
			// Request the targeted Fault Counter reset.
			//
			UARTSendMessage(LM_API_STATUS_FLT_COUNT | g_ulID, &ucData, 1);
		} else {

			//
			// Request the current Fault Counters.
			//
			UARTSendMessage(LM_API_STATUS_FLT_COUNT | g_ulID, 0, 0);
		}

		WaitForAck(LM_API_ACK | g_ulID, 10);
	}

	else {
		printf("%s [vout|vbus|fault|cur|temp|pos|speed|limit|power|cmode|\n"
				"\tvout2|stkyfault|faultcnts]\n", argv[0]);
	}

	return (0);
}

//*****************************************************************************
//
// This command is used to set configuration parameters used by the devices.
//
//*****************************************************************************
int fastConfigTurns(int32_t numTurns) {
	int32_t plValue[2];

	//
	// set the number of turns in a potentiometer based on the
	// parameters.
	//
        plValue[0] = numTurns;

	//
	// Limit the value to valid settings.
	//
	if (plValue[0] < 0) {
		plValue[0] = 0;
	}

	if (plValue[0] > 65535) {
		plValue[0] = 65535;
	}

	UARTSendMessage(LM_API_CFG_POT_TURNS | g_ulID, (unsigned char *) plValue, 2);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

int CmdConfig(int argc, char *argv[]) {
	int32_t plValue[2];

	else if ((argc > 1) && (strcmp(argv[1], "limit") == 0)) {
		//
		// If there is a second argument then this is a set request.
		//
		if (argc > 2) {
			if (strcmp(argv[2], "off") == 0) {
				//
				// Disable the limit switches.
				//
				plValue[0] = 0;
			} else if (strcmp(argv[2], "on") == 0) {
				//
				// Enable the limit switches.
				//
				plValue[0] = 1;
			} else {
				printf("%s %s [on|off]\n", argv[0], argv[1]);
				return (0);
			}

			UARTSendMessage(LM_API_CFG_LIMIT_MODE | g_ulID,
					(unsigned char *) plValue, 1);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// This is a request to retrieve the value.
			//
			UARTSendMessage(LM_API_CFG_LIMIT_MODE | g_ulID, 0, 0);

			WaitForAck(LM_API_CFG_LIMIT_MODE | g_ulID, 10);
		}
	}

	else if ((argc > 1) && (strcmp(argv[1], "fwd") == 0)) {
		//
		// If there is enough data then this is a request to set the value.
		//
		if (argc > 3) {
			//
			// Get the position limit.
			//
			plValue[0] = strtol(argv[2], 0, 0);

			//
			// Check if the value is a less than or greater than some position.
			//
			if (strcmp(argv[3], "lt") == 0) {
				plValue[1] = 0;
			} else if (strcmp(argv[3], "gt") == 0) {
				plValue[1] = 1;
			} else {
				printf("%s %s <pos> [lt|gt]\n", argv[0], argv[1]);
				return (0);
			}

			UARTSendMessage(LM_API_CFG_LIMIT_FWD | g_ulID,
					(unsigned char *) plValue, 5);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// This is a request to retrieve the value.
			//
			UARTSendMessage(LM_API_CFG_LIMIT_FWD | g_ulID, 0, 0);

			WaitForAck(LM_API_CFG_LIMIT_FWD | g_ulID, 10);
		}
	}

	else if ((argc > 1) && (strcmp(argv[1], "rev") == 0)) {
		//
		// If there is enough data then this is a request to set the value.
		//
		if (argc > 3) {
			//
			// Get the position limit.
			//
			plValue[0] = strtol(argv[2], 0, 0);

			//
			// Check if the value is a less than or greater than some position.
			//
			if (strcmp(argv[3], "lt") == 0) {
				plValue[1] = 0;
			} else if (strcmp(argv[3], "gt") == 0) {
				plValue[1] = 1;
			} else {
				printf("%s %s <pos> [lt|gt]\n", argv[0], argv[1]);
				return (0);
			}

			UARTSendMessage(LM_API_CFG_LIMIT_REV | g_ulID,
					(unsigned char *) plValue, 5);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// This is a request to retrieve the value.
			//
			UARTSendMessage(LM_API_CFG_LIMIT_REV | g_ulID, 0, 0);

			WaitForAck(LM_API_CFG_LIMIT_REV | g_ulID, 10);
		}
	}

	else if ((argc > 1) && (strcmp(argv[1], "maxvout") == 0)) {
		//
		// If there is a second argument then this is a set request.
		//
		if (argc > 2) {
			//
			// Get the Max voltage out setting.
			//
			plValue[0] = strtol(argv[2], 0, 0);

			//
			// Limit the value to valid settings.
			//
			if (plValue[0] < 0) {
				plValue[0] = 0;
			}

			if (plValue[0] > (12 * 256)) {
				plValue[0] = 12 * 256;
			}

			UARTSendMessage(LM_API_CFG_MAX_VOUT | g_ulID,
					(unsigned char *) plValue, 2);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// This is a request to retrieve the value.
			//
			UARTSendMessage(LM_API_CFG_MAX_VOUT | g_ulID, 0, 0);

			WaitForAck(LM_API_CFG_MAX_VOUT | g_ulID, 10);
		}
	}

	else if ((argc > 1) && (strcmp(argv[1], "faulttime") == 0)) {
		//
		// If there is a second argument then this is a set request.
		//
		if (argc > 2) {
			//
			// Get the fault timeout value to set.
			//
			plValue[0] = strtol(argv[2], 0, 0);

			//
			// Limit the value to valid settings.
			//
			if (plValue[0] < 0) {
				plValue[0] = 0;
			}
			if (plValue[0] > 65535) {
				plValue[0] = 65535;
			}

			UARTSendMessage(LM_API_CFG_FAULT_TIME | g_ulID,
					(unsigned char *) plValue, 2);

			WaitForAck(LM_API_ACK | g_ulID, 10);
		} else {
			//
			// This is a request to retrieve the value.
			//
			UARTSendMessage(LM_API_CFG_FAULT_TIME | g_ulID, 0, 0);

			WaitForAck(LM_API_CFG_FAULT_TIME | g_ulID, 10);
		}
	}

	else {
		printf("%s [lines|turns|brake|limit|fwd|rev|maxvout|faulttime]\n",
				argv[0]);
	}

	return (0);
}

//
// This command handler takes care of the system level commands.
//
//*****************************************************************************
int CmdSystem(int argc, char *argv[]) {
	uint32_t ulValue;
	char pcBuffer[8];

	if ((argc > 1) && (strcmp(argv[1], "halt") == 0)) {
		//
		// Broadcast a Halt command.
		//
		UARTSendMessage(CAN_MSGID_API_SYSHALT, 0, 0);
	} else if ((argc > 1) && (strcmp(argv[1], "resume") == 0)) {
		//
		// Broadcast a Resume command.
		//
		UARTSendMessage(CAN_MSGID_API_SYSRESUME, 0, 0);
	} else if ((argc > 1) && (strcmp(argv[1], "reset") == 0)) {
		//
		// Broadcast a system reset command.
		//
		UARTSendMessage(CAN_MSGID_API_SYSRST, 0, 0);
	} else if ((argc > 1) && (strcmp(argv[1], "enum") == 0)) {
		//
		// Broadcast a system enumeration command.
		//
		UARTSendMessage(CAN_MSGID_API_ENUMERATE, 0, 0);

		//
		// Wait for a device query response.
		//
		WaitForAck(CAN_MSGID_API_DEVQUERY | g_ulID, 100);
	} else if ((argc > 1) && (strcmp(argv[1], "query") == 0)) {
		//
		// Handle the device query command that will return information about
		// the device.
		//
		UARTSendMessage(CAN_MSGID_API_DEVQUERY | g_ulID, 0, 0);

		WaitForAck(CAN_MSGID_API_DEVQUERY | g_ulID, 10);
	} else if ((argc > 1) && (strcmp(argv[1], "sync") == 0)) {
		//
		// Send out a synchronous update command.
		//
		if (argc > 2) {
			//
			// Get the synchronous update ID from the parameter.
			//
			ulValue = strtoul(argv[2], 0, 0);

			UARTSendMessage(CAN_MSGID_API_SYNC, (unsigned char *) &ulValue, 1);
		} else {
			printf("%s %s <group>\n", argv[0], argv[1]);
		}
	}

	return (0);
}


//*****************************************************************************
//
// Handle shutting down the applcation.
//
//*****************************************************************************
int CmdExit(int argc, char *argv[]) {
	CloseUART();
	exit(0);
}


//*****************************************************************************
//
// Finds the Jaguars on the network.
//
//*****************************************************************************
void FindJaguars(void) {
	int iIdx;

	//
	// Initialize the status structure list to all 0's.
	//
	for (iIdx = 0; iIdx < MAX_CAN_ID; iIdx++) {
		g_sBoardState[iIdx].ulControlMode = LM_STATUS_CMODE_VOLT;
	}

	//
	// Send the enumerate command.
	//
	strcpy(g_argv[0], "system");
	strcpy(g_argv[1], "enum");
	CmdSystem(2, g_argv);
}

