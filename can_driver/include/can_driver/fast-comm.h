//*****************************************************************************
//
// fast-comm.h header for all the fast commands instead of using argv and argc
//
//*****************************************************************************

#ifndef FAST_COMM_H
#define FAST_COMM_H
int fastCmdID(uint32_t jagId);
int fastCmdHeartbeat();
int fastCmdPosEnable(uint32_t jagId, int32_t inpValue);
int fastCmdPosDis(uint32_t jagId);
int fastCmdPosSet(uint32_t jagId,int32_t inpValue);
int fastCmdPosP(uint32_t jagId,int32_t inpValue);
int fastCmdPosI(uint32_t jagId,int32_t inpValue);
int fastCmdPosD(uint32_t jagId,int32_t inpValue);
int fastCmdPosRef(uint32_t jagId,int32_t inpValue);
int fastCmdPosSetNoAck(uint32_t jagId,int32_t inpValue);
int fastConfigTurns(uint32_t jagId,int32_t numTurns);
int fastConfigMaxV(uint32_t jagId,int32_t maxV);
int fastSystemHalt(void);
int fastSystemResume(void);
int fastSystemReset(void);
int fastSystemEnum(uint32_t jagId);
int fastSystemQuery(void);
int fastSystemSync(uint32_t syncGrp);
#endif
