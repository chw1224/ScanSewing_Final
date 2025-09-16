//----------------------------------------------------------------------------
// HostBinRot.h
// --------------
//
// Includes for host port rotary buffer download function.
//
//
//----------------------------------------------------------------------------

#ifndef _HOSTOTLIB_H
  #define _HOSTROTLIB_H

//-------------------------------------------------------------------------
DWORD CALLBACK PackDPRRotDataToHostFormat(int num_put,PBYTE rotdatin,PBYTE rotdatout);
//-------------------------------------------------------------------------
SHORT  CALLBACK PmacHostrotput(DWORD dwDevice,int num_put,LPDWORD rotdat,int bufnum,BOOL bSendImmediate,PCHAR inpstr);
//-------------------------------------------------------------------------
void CALLBACK PmacHost24Flush( DWORD dwDevice );
//-------------------------------------------------------------------------
void CALLBACK PmacHostBinRotInit( DWORD dwDevice );
//-------------------------------------------------------------------------
void CALLBACK PmacHostBkgMode( DWORD dwDevice );
//-------------------------------------------------------------------------
void CALLBACK PmacHostRTMode( DWORD dwDevice );
//-------------------------------------------------------------------------
void CALLBACK SetBinRotHostMode(DWORD dwDevice,BOOL bOn);

#endif