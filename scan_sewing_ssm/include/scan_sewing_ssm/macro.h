#ifndef _MACRO_H
  #define _MACRO_H

typedef struct {
  DWORD dwNumActiveNodes;// Number of MACRO Stations enabled on UltraLight
  DWORD dwNumMACROStations;// Number of MACRO Stations enabled on UltraLight
  DWORD dwNodeActive[64];  // Node enabled on Ultra-Light?
  DWORD dwIsIONode[64];    // 1 = I/O -- 0 = SERVO
  DWORD dwMACROStationID[64];// STATION ID (MI3) Associated with Node at at same index
} ULTRALIGHT_INFO;

// Functions --------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

  BOOL CALLBACK MACROSetIVariableString( DWORD dwDevice, DWORD node, UINT num, char *szMIVarValue);
  PCHAR CALLBACK MACROGetIVariableString( DWORD dwDevice, DWORD node, UINT num, PCHAR def );
  long CALLBACK MACROGetIVariableLong( DWORD dwDevice, DWORD node, UINT num, long def );
  BOOL CALLBACK MACROUploadConfig( DWORD dwDevice, DOWNLOADPROGRESS prgp, char *fname );
  BOOL MACROGetIVariableSet( DWORD dwDevice, DOWNLOADPROGRESS progressProc,
       long node, FILE * fp, long start, long end );
  BOOL MACROUploadGlobalNodeConfig( DWORD dwDevice, DOWNLOADPROGRESS prgp, long LowNode, FILE * fp );
  void GetActiveServoNodes( DWORD dwDevice );
  DWORD CALLBACK UltraLightAnalyze(DWORD dwDevice,ULTRALIGHT_INFO * ui);

#ifdef __cplusplus
}
#endif
#endif
