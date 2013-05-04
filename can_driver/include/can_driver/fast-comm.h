//*****************************************************************************
//
// fast-comm.h header for all the fast commands instead of using argv and argc
//
//*****************************************************************************

#ifndef FAST_COMM_H
#define FAST_COMM_H
int fastCmdID(uint32_t jagId);
int fastCmdHeartbeat();
int fastCmdPosEnable(int32_t inpValue);
int fastCmdPosDis(void);
int fastCmdPosSet(int32_t inpValue);
int fastCmdPosP(int32_t inpValue);
int fastCmdPosI(int32_t inpValue);
int fastCmdPosD(int32_t inpValue);
int fastCmdPosRef(int32_t inpValue);
int fastCmdPosSetNoAck(int32_t inpValue);
int fastConfigTurns(int32_t numTurns);
int fastConfigMaxV(int32_t maxV);
int fastSystemHalt(void);
int fastSystemResume(void);
int fastSystemReset(void);
int fastSystemEnum(void);
int fastSystemQuery(void);
int fastSystemSync(uint32_t syncGrp);
void fastFindJaguars(void);
#endif
