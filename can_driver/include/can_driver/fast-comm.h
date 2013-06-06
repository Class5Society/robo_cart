//*****************************************************************************
//
// fast-comm.h header for all the fast commands instead of using argv and argc
//
//*****************************************************************************

#ifndef FAST_COMM_H
#define FAST_COMM_H
int fastCmdID(uint32_t jagId);
int fastCmdHeartbeat();
int fastCmdPosEnable(uint32_t jagId, double inpPosValue);
int fastCmdPosDis(uint32_t jagId);
double fastCmdPosSet(uint32_t jagId, double inpPosValue);
double fastCmdPosGet(uint32_t jagId);
int fastCmdPosP(uint32_t jagId, double inpPValue);
int fastCmdPosI(uint32_t jagId, double inpIValue);
int fastCmdPosD(uint32_t jagId, double inpDValue);
int fastCmdPosRef(uint32_t jagId,int32_t inpValue);
int fastCmdPosSetNoAck(uint32_t jagId, double inpPosValue);
int fastConfigTurns(uint32_t jagId,int32_t numTurns);
int fastConfigMaxV(uint32_t jagId, double inpMaxV);
int fastSystemHalt(void);
int fastSystemResume(void);
int fastSystemReset(void);
int fastSystemEnum(uint32_t jagId);
int fastSystemQuery(void);
int fastSystemSync(uint32_t syncGrp);
double fastParseResponse(void);
int fastWaitForAck(uint32_t ulID, uint32_t ulTimeout);

#endif
