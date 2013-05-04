//*****************************************************************************
//
// gui_handlers.cxx - GUI handler functions.
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
// The current board status and board states for all devices.
//
//*****************************************************************************
tBoardStatus g_sBoardStatus;
tBoardState g_sBoardState[MAX_CAN_ID];

//*****************************************************************************
//
// Local module global to manage PStat History Legend buffer
//
//*****************************************************************************
static void *pvPeriodicStatusLegendBuffer = NULL;

//*****************************************************************************
//
// This array holds the strings used for periodic status messages in the GUI.
//
//*****************************************************************************
tPStatMsgEncodes g_sPStatMsgs[] =
{
    { "END MSG", "msg-end", LM_PSTAT_END },
    { "VOUT B0 (%)", "vout-b0", LM_PSTAT_VOLTOUT_B0 },
    { "VOUT B1 (%)", "vout-b1", LM_PSTAT_VOLTOUT_B1 },
    { "VBUS B0", "vbus-b0", LM_PSTAT_VOLTBUS_B0 },
    { "VBUS B1", "vbut-b1", LM_PSTAT_VOLTBUS_B1 },
    { "Current B0", "curr-b0", LM_PSTAT_CURRENT_B0 },
    { "Current B1", "curr-b1", LM_PSTAT_CURRENT_B1 },
    { "Temp B0", "temp-b0", LM_PSTAT_TEMP_B0 },
    { "Temp B1", "temp-b1", LM_PSTAT_TEMP_B1 },
    { "Pos B0", "pos-b0", LM_PSTAT_POS_B0 },
    { "Pos B1", "pos-b1", LM_PSTAT_POS_B1 },
    { "Pos B2", "pos-b2", LM_PSTAT_POS_B2 },
    { "Pos B3", "pos-b3", LM_PSTAT_POS_B3 },
    { "Speed B0", "spd-b0", LM_PSTAT_SPD_B0 },
    { "Speed B1", "spd-b1", LM_PSTAT_SPD_B1 },
    { "Speed B2", "spd-b2", LM_PSTAT_SPD_B2 },
    { "Speed B3", "spd-b3", LM_PSTAT_SPD_B3 },
    { "Limit (NoCLR)", "lim-nclr", LM_PSTAT_LIMIT_NCLR },
    { "Limit (CLR)", "lim-clr", LM_PSTAT_LIMIT_CLR },
    { "Fault", "fault", LM_PSTAT_FAULT },
    { "Stky Fault (NoCLR)", "sfault-nclr", LM_PSTAT_STKY_FLT_NCLR },
    { "Stky Fault (CLR)", "sfault-clr", LM_PSTAT_STKY_FLT_CLR },
    { "VOUT B0 (V)", "vout2-b0", LM_PSTAT_VOUT_B0 },
    { "VOUT B1 (V)", "vout2-b1", LM_PSTAT_VOUT_B1 },
    { "Current Faults", "flt-curr", LM_PSTAT_FLT_COUNT_CURRENT },
    { "Temp Faults", "flt-temp", LM_PSTAT_FLT_COUNT_TEMP },
    { "Voltage Faults", "flt-vbus", LM_PSTAT_FLT_COUNT_VOLTBUS },
    { "Gate Faults", "flt-gate", LM_PSTAT_FLT_COUNT_GATE },
    { "Comm Faults", "flt-comm", LM_PSTAT_FLT_COUNT_COMM },
    { "CAN Status", "cansts", LM_PSTAT_CANSTS },
    { "CAN RxErr", "canerr-b0", LM_PSTAT_CANERR_B0 },
    { "CAN TXErr", "canerr-b1", LM_PSTAT_CANERR_B1 },
    { 0, 0, 0 }
};


//*****************************************************************************
//
// Strings for the PStat legend.
//
//*****************************************************************************
static const char *g_ppcLegentTitles[] =
{
    "   Vout    ",
    "   Vbus    ",
    "  Current  ",
    "Temperature",
    " Position  ",
    "   Speed   ",
    "Curr_Faults",
    "Temp Faults",
    "Vbus Faults",
    "Gate Faults",
    "Comm Faults",
    "CAN Status ",
    "CAN_RX Err ",
    "CAN TX Err ",
    "   Limit   ",
    "   Faults  ",
    0
};

