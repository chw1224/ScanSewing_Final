//----------------------------------------------------------------------------
//  Project PMAC ETHERNET 
//  Delta Tau Data Systems, Inc.
//  Copyright ?2001. All Rights Reserved.
//
//  SUBSYSTEM:    Ethernet I/O Interface Example
//  FILE:         PMACSOCK.CPP
//  AUTHOR:       Delta Tau Data Systems, Inc.
//
//  OVERVIEW
//  ~~~~~~~~
//  Communication routine to talk to PMAC on ethernet
//
//----------------------------------------------------------------------------
#ifndef _PMAC_SOCK
#define _PMAC_SOCK

#include "winsock2.h"


#define MAX_PMAC_LINE        255
#define FLUSH_TIMEOUT         10 
#define PMACPORT            1025             // Port used for our TCP comm


   // Open a socket for 
HANDLE CALLBACK PmacSockOpen(DWORD dwDevice,DWORD IPAddress) ;
int CALLBACK PmacSockClose(DWORD dwDevice);
int CALLBACK PmacSockIPAddress( DWORD dwDevice,BOOL bSetIP ,BYTE *ip);
SOCKET CALLBACK ConvertInitialProcSocket(SOCKET initialSocket,DWORD initialPID);
 
   // Ascii comm routines
int  CALLBACK PmacSockFlush(DWORD dwDevice);
int  CALLBACK PmacSockSendLine(DWORD dwDevice, LPCSTR outstr );
int  CALLBACK PmacSockSendLineW(DWORD dwDevice,  PWCHAR outstr );
void CALLBACK PmacSockSendCommand( DWORD dwDevice, PCHAR outstr );
void CALLBACK PmacSockSendCommandW( DWORD dwDevice, PWCHAR outstr );
int  CALLBACK PmacSockGetLine( DWORD dwDevice, LPCSTR outstr, int maxChar, UINT *inChars );
int  CALLBACK PmacSockGetLineW(DWORD dwDevice,PWCHAR s, int maxchar, UINT *nc );
int  CALLBACK PmacSockGetBuffer( DWORD dwDevice, LPCSTR outstr, int size, UINT *nc );
int  CALLBACK PmacSockGetBufferW( DWORD dwDevice, PWCHAR outstr, int size, UINT *nc );
BOOL CALLBACK PmacSockSendCharA(  DWORD dwDevice, CHAR outch );
BOOL CALLBACK PmacSockSendCharW(DWORD dwDevice,  WCHAR outch );
int  CALLBACK PmacSockGetResponse( DWORD dwDevice, PCHAR s, UINT maxchar, PCHAR outstr );
int  CALLBACK PmacSockGetResponseW( DWORD dwDevice, PWCHAR s, UINT maxchar, PWCHAR outstr );
int  CALLBACK PmacSockGetLineError(DWORD dwDevice);

int  CALLBACK PmacSockGetLastError(DWORD dwDevice);
int  CALLBACK PmacSockOutport(DWORD dwDevice, BYTE offset, BYTE outch);
int  CALLBACK PmacSockInport(DWORD dwDevice, BYTE offset,BYTE *data);
BYTE CALLBACK PmacSockReadReady(DWORD dwDevice);
int  CALLBACK PmacSockGetControlResponse(DWORD dwDevice, CHAR *outstr, WORD len ,BYTE outchar );
int  CALLBACK PmacSockGetControlResponseW( DWORD dwDevice, PWCHAR s, UINT maxchar,WCHAR outchar );
int  CALLBACK PmacSockWriteBuffer(DWORD dwDevice, CHAR *data, WORD len );
int  CALLBACK PmacSockWriteBufferError( DWORD dwDevice, WORD *erroroffset, WORD *errorcode );
int  CALLBACK PmacSockWriteFWBuffer(DWORD dwDevice,BOOL bRestart, CHAR *data, DWORD len );


   // DPRAM routines
int CALLBACK PmacSockGetMem( DWORD dwDevice, BYTE *data, WORD offset,WORD length);
int CALLBACK PmacSockSetMem( DWORD dwDevice, BYTE *data, WORD offset,WORD length);
int CALLBACK PmacSockSetMemBit(DWORD dwDevice,DWORD offset,UINT bitno,BOOL on);
int CALLBACK PmacSockSetMemBits(DWORD dwDevice,DWORD offset,UINT startbit,UINT width,DWORD val);
BOOL CALLBACK PmacSockGetMemBit(DWORD dwDevice,DWORD offset,UINT bit,PBOOL val);
BOOL CALLBACK PmacSockGetMemBits(DWORD dwDevice,DWORD offset,UINT startbit,UINT width,PDWORD val);


  // DPRAM rotary buffer routines
int CALLBACK PmacSockRotBufInit(DWORD dwDevice, BYTE coord);
int CALLBACK PmacSockRotBufPut(DWORD dwDevice, BYTE coord,BYTE *data,
                               WORD len,WORD *erroroffset,WORD *errcode);

  

char * UnsolicitedPresent(char * data);

#endif 
