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
	g_ulID = jagId;

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
// This command enables the Position control mode.
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

//*****************************************************************************
//
// This command disables the Position control mode.
//
//*****************************************************************************
int fastCmdPosDis(void)
{
	//
	// Disable Position control mode.
	//
	UARTSendMessage(LM_API_POS_DIS | g_ulID, 0, 0);

	WaitForAck(LM_API_ACK | g_ulID, 10);

}

//*****************************************************************************
//
// This command sets the position location
//
//*****************************************************************************
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


//*****************************************************************************
//
// This command sets the position P value for PID.
//
//*****************************************************************************
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

//*****************************************************************************
//
// This command sets the position I value for PID.
//
//*****************************************************************************
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

//*****************************************************************************
//
// This command sets the position D value for PID.
//
//*****************************************************************************
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

//*****************************************************************************
//
// This command sets the position reference (Encoder =0 and Potentiometer = 1)
//
//*****************************************************************************
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

//*****************************************************************************
//
// This command sets the position location without waiting for an ack
//
//*****************************************************************************
int fastCmdPosSetNoAck(int32_t inpValue)
{
	int32_t plValue[2];

	//
	// Set Position.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_SET_NO_ACK | g_ulID, (unsigned char *) plValue, 4);

}


//*****************************************************************************
//
// This sets the number of turns on the potentionmeter.
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

//*****************************************************************************
//
// This sets the max percentage of voltage out
//
//*****************************************************************************
int fastConfigMaxV(int32_t maxV) {
	int32_t plValue[2];

	// Get the Max voltage out setting.
	//
	plValue[0] = maxV;

	//
        // Limit the value to valid settings.
        //
	if (plValue[0] < 0) {
		plValue[0] = 0;
	}

	if (plValue[0] > (12 * 256)) {
		plValue[0] = 12 * 256;
	}

	UARTSendMessage(LM_API_CFG_MAX_VOUT | g_ulID, (unsigned char *) plValue, 2);
	WaitForAck(LM_API_ACK | g_ulID, 10);
}

//*****************************************************************************
//
// This command handler takes care of the system level commands.
//
//*****************************************************************************

//*****************************************************************************
//
// This command handler takes care of the system level commands.
//
//*****************************************************************************

//*****************************************************************************
//
// This commands everthing to halt
//
//*****************************************************************************
int fastSystemHalt(void)
{
   UARTSendMessage(CAN_MSGID_API_SYSHALT, 0, 0);
}

//*****************************************************************************
//
// This commands everthing to resume 
//
//*****************************************************************************
int fastSystemResume(void)
{
    UARTSendMessage(CAN_MSGID_API_SYSRESUME, 0, 0);
}

//*****************************************************************************
//
// This commands everthing to reset
//
//*****************************************************************************
int fastSystemReset(void)
{
    UARTSendMessage(CAN_MSGID_API_SYSRST, 0, 0);
}

//*****************************************************************************
//
// This commands everthing to enumerate
//
//*****************************************************************************
int fastSystemEnum(void)
{
     //
     // Broadcast a system enumeration command.
     //
     UARTSendMessage(CAN_MSGID_API_ENUMERATE, 0, 0);

     //
     // Wait for a device query response.
     //
     WaitForAck(CAN_MSGID_API_DEVQUERY | g_ulID, 100);
}

//*****************************************************************************
//
// This commands everthing a query
//
//*****************************************************************************
int fastSystemQuery(void)
{
	//
	// Handle the device query command that will return information about
	// the device.
	//
	UARTSendMessage(CAN_MSGID_API_DEVQUERY | g_ulID, 0, 0);

	WaitForAck(CAN_MSGID_API_DEVQUERY | g_ulID, 10);
}


//*****************************************************************************
//
// This commands everthing to sync
//
//*****************************************************************************
int fastSystemSync(uint32_t syncGrp)
{
	uint32_t ulValue;

	//
	// Send out a synchronous update command.
	//
	//
	// Get the synchronous update ID from the parameter.
	//
	ulValue = syncGrp;

	UARTSendMessage(CAN_MSGID_API_SYNC, (unsigned char *) &ulValue, 1);
}

//*****************************************************************************
//
// Finds the Jaguars on the network.
//
//*****************************************************************************
void fastFindJaguars(void) {
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
        fastSystemEnum();
}

