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
#include "ros/ros.h"

//*****************************************************************************
//
// These variables are used to hold the messages as they come in from the UART.
//
//*****************************************************************************
static unsigned char gUARTMessage[12];
static uint32_t gUARTMessageSize;
static uint32_t gUARTMessageLenRead;

//*****************************************************************************
//
// The current UART state and its global variable.
//
//*****************************************************************************
static uint32_t gUARTMessageState = UART_STATE_IDLE;

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
	jagId = jagId;

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
int fastCmdPosEnable(uint32_t jagId, double inpPosValue)
{
	int32_t plValue[2];
        int32_t inpValue;

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpPosValue);

	//
	// Enable Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_EN | jagId, (unsigned char *) plValue, 4);

	fastWaitForAck(LM_API_ACK | jagId, 10);

}

//*****************************************************************************
//
// This command disables the Position control mode.
//
//*****************************************************************************
int fastCmdPosDis(uint32_t jagId)
{
	//
	// Disable Position control mode.
	//
	UARTSendMessage(LM_API_POS_DIS | jagId, 0, 0);

	fastWaitForAck(LM_API_ACK | jagId, 10);

}

//*****************************************************************************
//
// This command sets the position location
//
//*****************************************************************************
double fastCmdPosSet(uint32_t jagId, double inpPosValue)
{
        int32_t inpValue;
	int32_t plValue[2];
        int retVal = 0;
        double posVal = 0;

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpPosValue);

	//
	// Set Position.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_SET | jagId, (unsigned char *) plValue, 4);
	retVal = fastWaitForAck(LM_API_ACK | jagId, 10);
        
        //check for timeout
        if (retVal == 0)
        {
          //get the value from the message
          posVal = fastParseResponse();
          return(posVal);
        }
        else
        {
          //return the input value
          return(inpPosValue);           
        }

}


//*****************************************************************************
//
// This command sets the position P value for PID.
//
//*****************************************************************************
int fastCmdPosP(uint32_t jagId, double inpPValue)
{
	int32_t plValue[2];
        int32_t inpValue;

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpPValue);

	//
	// P value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_PC | jagId, (unsigned char *) plValue, 4);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}

//*****************************************************************************
//
// This command sets the position I value for PID.
//
//*****************************************************************************
int fastCmdPosI(uint32_t jagId, double inpIValue)
{
	int32_t plValue[2];
        int32_t inpValue;

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpIValue);


	//
	// Set the I value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_IC | jagId, (unsigned char *) plValue, 4);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}

//*****************************************************************************
//
// This command sets the position D value for PID.
//
//*****************************************************************************
int fastCmdPosD(uint32_t jagId, double inpDValue)
{
	int32_t plValue[2];
        int32_t inpValue;

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpDValue);


	//
	// Set the D value in Position control mode.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_DC | jagId, (unsigned char *) plValue, 4);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}

//*****************************************************************************
//
// This command sets the position reference (Encoder =0 and Potentiometer = 1)
//
//*****************************************************************************
int fastCmdPosRef(uint32_t jagId, int32_t inpValue)
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
	UARTSendMessage(LM_API_POS_REF | jagId, (unsigned char *) plValue, 1);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}

//*****************************************************************************
//
// This command sets the position location without waiting for an ack
//
//*****************************************************************************
int fastCmdPosSetNoAck(uint32_t jagId, double inpPosValue)
{
        int32_t inpValue;
	int32_t plValue[2];

        //
        // Convert the position
        //
        inpValue = (int32_t) ((double) POS_SCALE_FACTOR * inpPosValue);

	//
	// Set Position.
	//
	plValue[0] = inpValue;

	UARTSendMessage(LM_API_POS_SET_NO_ACK | jagId, (unsigned char *) plValue, 4);

}

//*****************************************************************************
//
// This command get the position location
//
//*****************************************************************************
double fastCmdPosGet(uint32_t jagId)
{
	int32_t plValue[2];
        int retVal;
        double posVal;

	//
	// Request the current position.
	//
	UARTSendMessage(LM_API_STATUS_POS | jagId, 0, 0);


	retVal = fastWaitForAck(LM_API_STATUS_POS | jagId, 10);

        //check for timeout
        if (retVal == 0)
        {
          posVal = fastParseResponse();
          return(posVal);
        }
        else
        {
          //return the input value
          return(-1);           
        }

}


//*****************************************************************************
//
// This sets the number of turns on the potentionmeter.
//
//*****************************************************************************
int fastConfigTurns(uint32_t jagId, int32_t numTurns) {
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

	UARTSendMessage(LM_API_CFG_POT_TURNS | jagId, (unsigned char *) plValue, 2);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}

//*****************************************************************************
//
// This sets the max percentage of voltage out
//
//*****************************************************************************
int fastConfigMaxV(uint32_t jagId, double inpMaxV) {

	int32_t plValue[2];
        int32_t maxV;

        //
        // Convert the voltage
        //
        maxV = (int32_t) ((double) MAX_VOUT_SCALE_FACTOR * inpMaxV);

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

	UARTSendMessage(LM_API_CFG_MAX_VOUT | jagId, (unsigned char *) plValue, 2);
	fastWaitForAck(LM_API_ACK | jagId, 10);
}


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
int fastSystemEnum(uint32_t jagId)
{
     //
     // Broadcast a system enumeration command.
     //
     UARTSendMessage(CAN_MSGID_API_ENUMERATE, 0, 0);

     //
     // Wait for a device query response.
     //
     fastWaitForAck(CAN_MSGID_API_DEVQUERY | jagId, 100);
}

//*****************************************************************************
//
// This commands everthing a query
//
//*****************************************************************************
int fastSystemQuery(uint32_t jagId)
{
	//
	// Handle the device query command that will return information about
	// the device.
	//
	UARTSendMessage(CAN_MSGID_API_DEVQUERY | jagId, 0, 0);

	fastWaitForAck(CAN_MSGID_API_DEVQUERY | jagId, 10);
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
// Parse the UART response message from the network.
//
//*****************************************************************************
double fastParseResponse(void) {
	uint32_t ulID;
	int32_t lValue;
        uint16_t shortValue;
        uint8_t byteValue;
	double dValue = -1;
	int iDevice;

	//
	// Get the device number out of the first byte of the message.
	//
	ulID = *(uint32_t *) gUARTMessage;
	iDevice = ulID & CAN_MSGID_DEVNO_M;


	//
	// Read the actual command out of the message.
	//
	switch (ulID & ~(CAN_MSGID_DEVNO_M)) {
	//
	// Handle the device enumeration command.
	//

	//
	// Handle the Position control mode position set request.
	//
	case LM_API_POS_SET: {
		lValue = *(int32_t *) (gUARTMessage + 4);

                dValue = (double) lValue / POS_SCALE_FACTOR;

		break;
	}

	//
	// Handle the Position control mode P parameter set request.
	//
	case LM_API_POS_PC: {
		lValue = *(int32_t *) (gUARTMessage + 4);

                dValue = (double) lValue / POS_SCALE_FACTOR;

		break;
	}

	//
	// Handle the Position control mode I parameter set request.
	//
	case LM_API_POS_IC: {
		lValue = *(int32_t *) (gUARTMessage + 4);

                dValue = (double) lValue / POS_SCALE_FACTOR;

		break;
	}

	//
	// Handle the Position control mode D parameter set request.
	//
	case LM_API_POS_DC: {
		lValue = *(int32_t *) (gUARTMessage + 4);

                dValue = (double) lValue / POS_SCALE_FACTOR;

		break;
	}

	//
	// Handle the Position control mode position reference set request.
	//
	case LM_API_POS_REF: {
                byteValue = gUARTMessage[4];
                dValue = (double) byteValue;

		break;
	}

	//
	// Handle the get Position request.
	//
	case LM_API_STATUS_POS: {
		//
		// Grab the response data and store it into a single variable.
		//
		lValue = *(int32_t *) (gUARTMessage + 4);

		//
		// Update the window with the new value.
		//
                dValue = (double) lValue / POS_SCALE_FACTOR;


		break;
	}


	//
	// Handle the get Number of Pot Turns request.
	//
	case LM_API_CFG_POT_TURNS: {
                shortValue =  *(unsigned short *) (gUARTMessage + 4);
                dValue = (double) shortValue;

		break;
	}


	//
	// Handle the get Maximum Voltage out response.
	//
	case LM_API_CFG_MAX_VOUT: {
                shortValue =  *(unsigned short *) (gUARTMessage + 4);
                dValue = (double) shortValue;
		break;
	}
    }

    //return the value
    return(dValue);
}
//*****************************************************************************
//
// This function handles waiting for an ACK from the device, and includes a
// timeout.
//
//*****************************************************************************
int fastWaitForAck(uint32_t ulID, uint32_t ulTimeout) {
	unsigned char ucChar;

	while (1) {
		//
		// If the UART timed out or failed to read for some reason then just
		// return.
		//
		if (UARTReceiveData(&ucChar, 1) == -1) {
			if (--ulTimeout == 0) {
				return (-1);
			}
			continue;
		}

		//
		// See if this is a start of packet byte.
		//
		if (ucChar == 0xff) {
			//
			// Reset the length of the UART message.
			//
			gUARTMessageLenRead = 0;

			//
			// Set the state such that the next byte received is the size
			// of the message.
			//
			gUARTMessageState = UART_STATE_LENGTH;
		}

		//
		// See if this byte is the size of the message.
		//
		else if (gUARTMessageState == UART_STATE_LENGTH) {
			//
			// Save the size of the message.
			//
			gUARTMessageSize = ucChar;

			//
			// Subsequent bytes received are the message data.
			//
			gUARTMessageState = UART_STATE_DATA;
		}

		//
		// See if the previous character was an escape character.
		//
		else if (gUARTMessageState == UART_STATE_ESCAPE) {
			//
			// See if this 0xfe, the escaped version of 0xff.
			//
			if (ucChar == 0xfe) {
				//
				// Store a 0xff in the message buffer.
				//
				gUARTMessage[gUARTMessageLenRead++] = 0xff;

				//
				// Subsequent bytes received are the message data.
				//
				gUARTMessageState = UART_STATE_DATA;
			}

			//
			// Otherwise, see if this is 0xfd, the escaped version of 0xfe.
			//
			else if (ucChar == 0xfd) {
				//
				// Store a 0xfe in the message buffer.
				//
				gUARTMessage[gUARTMessageLenRead++] = 0xfe;

				//
				// Subsequent bytes received are the message data.
				//
				gUARTMessageState = UART_STATE_DATA;
			}

			//
			// Otherwise, this is a corrupted sequence.  Set the receiver
			// to idle so this message is dropped, and subsequent data is
			// ignored until another start of packet is received.
			//
			else {
				gUARTMessageState = UART_STATE_IDLE;
			}
		}

		//
		// See if this is a part of the message data.
		//
		else if (gUARTMessageState == UART_STATE_DATA) {
			//
			// See if this character is an escape character.
			//
			if (ucChar == 0xfe) {
				//
				// The next byte is an escaped byte.
				//
				gUARTMessageState = UART_STATE_ESCAPE;
			} else {
				//
				// Store this byte in the message buffer.
				//
				gUARTMessage[gUARTMessageLenRead++] = ucChar;
			}
		}

		//
		// See if the entire message has been received but has not been
		// processed (i.e. the most recent byte received was the end of the
		// message).
		//
		if ((gUARTMessageLenRead == gUARTMessageSize)
				&& (gUARTMessageState == UART_STATE_DATA)) {
			//
			// The UART interface is idle, meaning all bytes will be
			// dropped until the next start of packet byte.
			//
			gUARTMessageState = UART_STATE_IDLE;

			if (*(uint32_t *) gUARTMessage == ulID) {
				return (0);
			}
		}
	}
	return (0);
}

