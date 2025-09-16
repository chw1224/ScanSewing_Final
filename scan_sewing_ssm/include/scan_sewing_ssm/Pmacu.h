/* pmacu.h
 *
 * 32-bit Motion Control Device Driver
 * Data structures and function entry-points for Pmac user-mode driver

HISTORY

*/

#ifndef _PMACU_H_
  #define _PMACU_H_

#ifndef __ATLBASE_H__
  #include <winsock2.h>	// 2016.05.27. 안병건 추가
  #include <windows.h>
  #include <windowsx.h>
#endif

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0400
#endif

// include needed headers
//
  #include <tchar.h>
  #include <winsock2.h>
  #include <stdlib.h>
  #include "mcstruct.h"
  #include "common.h"
  #include "mioctl.h"
  #include "private.h"
//  #include "PlxApi.h"

#ifndef _PMACSERVER
//////////////////////////////////////////////////////////////////////////////
// This section is included for the Pcomm32.dll and old server
  #include "dpr.h"
  #include "intr.h"
  #include "gather.h"

/****************************************************************************
 * information needed to locate/initialise hardware. Set by user
 * in ConfigDlgProc and written to registry to kernel driver.
 ****************************************************************************/
#pragma pack(push, 4)
typedef struct _Config_Location 
{
  PMACDEVICETYPE PmacType; // 1 = DT_PMAC1, 2 = DT_PMAC2,3 = DT_TURBO
  DWORD Location;          // typedef enum {LT_UNKNOWN, LT_ISA, LT_SER, LT_ETH, LT_PCI, LT_USB, LT_LAST};
  DWORD PCBusType;         // ISA, PCI etc.
  DWORD DPRRotSize[MAX_ROTBUF_PMAC]; // 16 originally

  DWORD DPRVarAdr;

  DWORD  dwDPRRTimeout;
  DWORD  dwDPRBKTimeout;
  DWORD  dwDPRCharTimeout;
  DWORD  dwDPRFlushTimeout;
  DWORD  dwBUSFlushTimeout;
  DWORD  dwBUSCharTimeout;

  DWORD  dwSERPortNumber;
  DWORD  dwSERBaudrate;
  DWORD  dwSERParity;  // 0 = no, 1 = odd, 2 = even
  DWORD  dwSERFlushTimeout;
  DWORD  dwSERCharTimeout;
  DWORD  dwSERWriteMultiplyerTimeout;
  DWORD  dwSERWriteConstantTimeout;
  DWORD  dwSERReadMultiplyerTimeout;
  DWORD  dwSERReadConstantTimeout;
  DWORD  dwSERReadIntervalTimeout;

  DWORD  dwDPRAutoActive;
  DWORD  dwDPRRealtActive;
  DWORD  dwDRPBackgActive;
  DWORD  dwDPRRealtPeriod;
  DWORD  dwDRPBackgPeriod;
  DWORD  dwEnableInterrupts;

  DWORD  dwIPAddress;

  CHAR szNcTitle[256];
  CHAR szSourceProfile[_MAX_PATH];
  CHAR szToolProfile[_MAX_PATH];
  CHAR szCoordProfile[_MAX_PATH];
  DWORD MachineType;
  DWORD NoOfTools;
  DWORD NoOfBlocks;
  DWORD NoOfCoordSys;
  DWORD MetricDisplay;
  double LeastHandleInc;
  double MaxHandleInc;
  double LeastJogInc;
  CHAR szAxisMotorMap[15];
  CHAR szAxisMotorSel[15];
  CHAR szAxisDispMap[15];
  double MaxRapidOvrd;
  double MaxFeedOvrd;
  // Axis stuff
  DWORD IsSpindle;
  DWORD HasSlave;
  DWORD IsPhantom;
  DWORD Display;
  DWORD DisplaySlave;
  DWORD HomeMode;
  DWORD HomePrgNumber;
  DWORD Precision;
  DWORD AxisMetricDisplay;
  DWORD MetricUnits;
  DWORD ProbePrgNumber;
  CHAR szPulsePerUnit[15];
  CHAR szInPositionBand[15];
  CHAR szMaxRapid[15];
  CHAR szMaxFeed[15];
  CHAR szFatalFError[15];
  CHAR szWarnFError[15];
  CHAR szJogSpeedLow[15];
  CHAR szJogSpeedMedLow[15];
  CHAR szJogSpeedMed[15];
  CHAR szJogSpeedMedHigh[15];
  CHAR szJogSpeedHigh[15];
  CHAR szFormatInch[15];
  CHAR szFormatMM[15];
} CONFIG_LOCATION, *PCONFIG_LOCATION;
#pragma pack(pop) 
#else
//////////////////////////////////////////////////////////////////////////////
// This section is included for the new server

 /****************************************************************************
 * information needed to locate/initialise hardware. Set by user
 * in ConfigDlgProc and written to registry to kernel driver.
 ****************************************************************************/
#pragma pack(push, 4)
typedef struct _Config_Location 
{
  PMACDEVICETYPE PmacType; // 1 = DT_PMAC1, 2 = DT_PMAC2,3 = DT_TURBO
  DWORD Location;          // typedef enum {LT_UNKNOWN, LT_ISA, LT_SER, LT_ETH, LT_PCI, LT_USB, LT_LAST};
  DWORD PCBusType;         // ISA, PCI etc.
  DWORD DPRRotSize[MAX_ROTBUF_PMAC]; // 16 originally

  DWORD DPRVarAdr;

  DWORD  dwDPRRTimeout;
  DWORD  dwDPRBKTimeout;
  DWORD  dwDPRCharTimeout;
  DWORD  dwDPRFlushTimeout;
  DWORD  dwBUSFlushTimeout;
  DWORD  dwBUSCharTimeout;

  DWORD  dwSERPortNumber;
  DWORD  dwSERBaudrate;
  DWORD  dwSERParity;  // 0 = no, 1 = odd, 2 = even
  DWORD  dwSERFlushTimeout;
  DWORD  dwSERCharTimeout;
  DWORD  dwSERWriteMultiplyerTimeout;
  DWORD  dwSERWriteConstantTimeout;
  DWORD  dwSERReadMultiplyerTimeout;
  DWORD  dwSERReadConstantTimeout;
  DWORD  dwSERReadIntervalTimeout;

  DWORD  dwDPRAutoActive;
  DWORD  dwDPRRealtActive;
  DWORD  dwDRPBackgActive;
  DWORD  dwDPRRealtPeriod;
  DWORD  dwDRPBackgPeriod;
  DWORD  dwEnableInterrupts;

  DWORD  dwIPAddress;

  CHAR szNcTitle[256];
  CHAR szSourceProfile[_MAX_PATH];
  CHAR szToolProfile[_MAX_PATH];
  CHAR szCoordProfile[_MAX_PATH];
  DWORD MachineType;
  DWORD NoOfTools;
  DWORD NoOfBlocks;
  DWORD NoOfCoordSys;
  DWORD MetricDisplay;
  double LeastHandleInc;
  double MaxHandleInc;
  double LeastJogInc;
  CHAR szAxisMotorMap[15];
  CHAR szAxisMotorSel[15];
  CHAR szAxisDispMap[15];
  double MaxRapidOvrd;
  double MaxFeedOvrd;
  // Axis stuff
  DWORD IsSpindle;
  DWORD HasSlave;
  DWORD IsPhantom;
  DWORD Display;
  DWORD DisplaySlave;
  DWORD HomeMode;
  DWORD HomePrgNumber;
  DWORD Precision;
  DWORD AxisMetricDisplay;
  DWORD MetricUnits;
  DWORD ProbePrgNumber;
  CHAR szPulsePerUnit[15];
  CHAR szInPositionBand[15];
  CHAR szMaxRapid[15];
  CHAR szMaxFeed[15];
  CHAR szFatalFError[15];
  CHAR szWarnFError[15];
  CHAR szJogSpeedLow[15];
  CHAR szJogSpeedMedLow[15];
  CHAR szJogSpeedMed[15];
  CHAR szJogSpeedMedHigh[15];
  CHAR szJogSpeedHigh[15];
  CHAR szFormatInch[15];
  CHAR szFormatMM[15];
} CONFIG_LOCATION, *PCONFIG_LOCATION;
#pragma pack(pop) 
  #include <intr.h>
  #include "pmac.h"

#endif  // _PMACSERVER

  #ifdef _CUI_
    #define _MAX_PATH MAX_PATHNAME_LEN
  #endif

  #define LINKLIST_FILE "PMAC_LINKLIST.INI"

  #ifdef __cplusplus
extern "C" {
  #endif

// forward declarations
typedef struct _USER_HANDLE USER_HANDLE, *PUSER_HANDLE;
typedef struct _GLOBAL_HANDLE GLOBAL_HANDLE, *PGLOBAL_HANDLE;


/////////////////////////////////////////////////////////////////////////////
// Process User Handle Sructure
/////////////////////////////////////////////////////////////////////////////
typedef struct _USER_HANDLE
{

  DWORD         dwSize;                         // Keep track of structure size
  DWORD         dwUser;                         // User No. for this device
  HANDLE        hDriver;                        // Handle to ring zero driver or COM
  HANDLE        hMutex;                         // Global mutex handle
  WSAEVENT      hPmacSockEvent;                 // Ethernet Event Object
  HANDLE        hEthMutex;
  CHAR          *szPmacData;                    // Buffered data on ethernet
  CHAR          *pData;                         // pointer to buffer next get line
  CHAR          *pDataStart;
  DWORD         dwPmacSockLastErr;

  // Command logging stuff
  BOOL       dCommandLogging;                   // Command Logging Enabled
  CHAR      tcComandLoggingFilename[MAX_PATH]; // Command Logging File Name
  UINT       uMaxCommandLoggingFileSize;        // Command Logging Max. file size
  HANDLE     hCommandLoggingFileHandle;         // Command Logging File Handle
  CHAR       languageDll[_MAX_PATH];            // Language translation DLL name

  // LT_COM - SERIAL
  BOOL          bDoChecksums;          // This process wants checksums
  char          csum;                  // Used to retain Command Sum for Checksummed comm

  // LT_ISA & LT_PCI - DPRAM
  ASCIIMODE     ascii_comm;            // current mode of ascii comm
  BOOL          bDpramAvailable;       // TRUE if DPRAM is available
  HANDLE        hDPRMutex;             // Global DPRAM serialized access mutex handle
  struct cpane  DPRCPanel;             // control panel           (ofs = 0x000,size = 36)
  struct cpane  DPRCPanelTurbo;        // ?? Turbo control panel  (ofs = 0x000,size = 36)

  struct backgvarbuf_status *bgv_status[MAX_VBGBUF_USERS ]; // Background variable data
  DWORD  DPRrotbuffer[16];                                  // DPR Binary Rotary Buffer Start

  // Gather stuff
  double *pGatherData[MAXGATHERS2 ];   // Pointers to gathered data
  double *pRTGatherData;               // Pointer to gathered data

  // Interrupt Stuff
  // inter-thread sync data
  UINT   ThreadId;
  HANDLE hThread;
  HANDLE hCompleteEvent;
  BOOL   bTerminateThread;

  // worker thread data
  MCCALLBACK Callback;                // interrupt callback function
  DWORD  WaitResult;
  INTRBUFFER Buffer;

  HWND   hWnd; // Window handle of app for use w/ messages boxes
//  PLX_DEVICE_OBJECT pcieDevice;

  /////////////////////////////////
  // ADDED for the NEW PmacServer and not used by the
  // PComm32.dll or OLD PmacServer, !Do not remove these from
  // the end of the structure
#ifndef _PMACSERVER
  UINT   dummy1;
  HANDLE dummy2;
  HANDLE dummy3;
  BOOL   dummy4;
#else
  // Interrupt Stuff & inter-thread sync data
  UINT   uiINTRThreadId;
  HANDLE hINTRThread;
  HANDLE hINTRCompleteEvent;
  BOOL   bINTRTerminateThread;
#endif

} USER_HANDLE, *PUSER_HANDLE;

///////////////////////////////////////////////////////////////////////////////
// Global
///////////////////////////////////////////////////////////////////////////////
#pragma pack(push, 4)
typedef struct _GLOBAL_HANDLE
{
  DWORD dwSize;       // Keep track of structure size
  BOOL  bInitialized; // Indicates that the global memory has been setup

  CHAR   version[VERSION_BUFFER_SIZE]; // store version string
  CHAR   date[DATE_BUFFER_SIZE];       // store date string
  CHAR   errorstr[80];                 // stored last error string
  PMACDEVICETYPE PmacType;             // 0 = DT_PMAC, 1 = DT_TURBO
  PMACFIRMWARETYPE PmacFirmware;
  DWORD  cpu_type;
  BOOL   bIsTurbo;

  LOCATIONTYPE Location; // PCBUS, SERIALPORT
  DWORD  dwEnum;         // Location Enumeration

  DWORD  dwMaxMotors;    // Maximum motors on PMAC, 32 if turbo else 8
  DWORD  dwMaxIMPQVars;  // Maximum IMPQ Vars on PMAC, 8191 if turbo else 1023
                         // BUS
  short  BUSCommError;   // BUS ascii comm error current value

  // LT_COM - SERIAL
  DCB           dcb;
  COMMTIMEOUTS  cto;

  DWORD         SERCommError;
  DWORD         SERPort;
  DWORD         SERBaudrate;
  UCHAR         SERParity;
  BOOL          SERCTSOutput;
  DWORD         SERRTSControl;

  // Status storage
  DWORD         DPRbuf_start;       // PMAC/TURBO DPR Start Address
  BOOL          bDPRInitialized;    // DPR has been initialized
  USHORT        DPRAsciiActive;     // DPR ASCII comm is currently active

  DWORD         DPRRealtActive;     // DPR foreground active
  DWORD         DPRRealtPeriod;     // DPR foreground update period in msec
  UINT          DPRNumMotors;       // number of motor/coords (i59) or masked

  DWORD         DPRAutoActive;      // DPR Automatic functions may need to be disabled for USB

  DWORD         DPRBackgActive;     // DPR background active
  DWORD         DPRBackgPeriod;     // DPR background update period in msec
  DWORD         EnableInterrupts;   // Enables Interrupts for PCI cards

  USHORT        DPRBackgvActive;    // DPR Background variable data buffer active
  USHORT        DPRCpanelActive;    // DPR control panel active
  USHORT        DPRRotaryActive;    // DPR Binary rotary buffer active
  struct realt  DPRRTBuffer;           // real time buffer        (ofs = 0x00D,size = 516)
  struct realtTURBO DPRRTBufferTurbo;  // Turbo real time buffer  (ofs = 0x068,size = 1556)
  struct backg  DPRBGBuffer;           // background buffer       (ofs = 0x228,size = 1028)
  struct backgTURBO DPRBGBufferTurbo;  // Turbo background buffer (ofs = 0x674,size = 1704)

  short         DPRCommError;       // DPR ascii comm error current value
  DWORD         i10;                // current i10 parameter
  double        posscale[32 ];      // current ix08 * 32 scale factor
  double        velscale[32 ];      // current ix09 * 32 scale factor

  USHORT        DPRRotSize[16];     // Size of DPRAM binary rotary 1 & 2
  USHORT        DPRBGVBnum_usrs;
  USHORT        DPRBGVBmulti_user_mode;
  DWORD         DPRVarStart;        // Start of DPR var & end of DPR buffers

  BOOL  CanUsePlcc;            // indicates can use compiled plc
  BOOL  WantUsePlcc;           // Want to compile PLC
  BOOL  Downloading;           // indicates download thread working
  BOOL  bootstrap_mode;        // In bootstrap mode
  DWORD linkfile[MAXLINKS];    // PLCC PMAC Link addresses
  DWORD plccbuffer;            // PLCC Program Scheduler Start Addresses
  DWORD plccret;               // PLCC Program RET
  DWORD plccstart;             // PLCC Program Start
  DWORD plccsize;              // Size of PLCC Memory
  DWORD plccjmp;               // PLCC Program Scheduler Start
  DWORD servosize;             // Size of Open Servo Memory
  BOOL	bSmallTurbo;			// Turbo CEM Style small memory Turbo


  // Gather stuff
  double ulGatherSampleTime;                    // Sample gather time in msec
  UINT   uGatherPeriod;                         // I19 number servo cycles per sample
  DWORD  dwGatherMask;                          // I20 (determines #sources & types)
  DWORD  dwGatherMask2;
  UINT   uGatherSources;                        // Number of sources gathered
  UINT   uGatherSamples;                        // Number of samples gathered
  UINT   uGatherSampleLen;                      // Number 24-bit words per sample
  BOOL   bGatherEnabled[MAXGATHERS2];           // Sources enabled
  char   szGatherAdr[MAXGATHERS2 ][MAXADRLEN ]; // Types and addresses of gathers
  UINT   uGatherSize[MAXGATHERS2];              // Size of gather type in 24bit words

  // Real gather stuff
  int    hRTGatherHandle;             // Handle to var background
  UINT   uRTGatherSources;            // Number of sources gathered
  UINT   uRTGatherSamples;            // Number of samples in gather array
  ULONG  ulRTGatherAdr[MAXRTGATHERS]; // Array of var buf gather sources

  // Mutex handle makes multi threading safe even from serial port
  //HANDLE hMutex;

  // Timeout values and counts
  DWORD  dwBaseTimeoutCount;

  DWORD  dwDPRRTimeoutCount;
  DWORD  dwDPRRTimeout;
  DWORD  dwDPRBKTimeoutCount;
  DWORD  dwDPRBKTimeout;
  DWORD  dwDPRCharTimeoutCount;
  DWORD  dwDPRCharTimeout;
  DWORD  dwDPRFlushTimeoutCount;
  DWORD  dwDPRFlushTimeout;
  DWORD  dwBUSCharTimeoutCount;
  DWORD  dwBUSCharTimeout;
  DWORD  dwSERFlushTimeout;
  DWORD  dwSERFlushTimeoutCount;
  DWORD  dwSERCharTimeout;
  DWORD  dwSERCharTimeoutCount;

  SOCKET initialSocket;
#ifndef WIN64
  SOCKET Dummy0;
#endif
  DWORD  initialPID;
  HANDLE initialSerial;
#ifndef WIN64
  HANDLE Dummy1;
#endif
  DWORD  initialSerPID;
  WSAPROTOCOL_INFO ProtocolInfo;
  DWORD  dwUserCount;

  DWORD  dwMaxDPRBKCoord;
  DWORD  dwMaxDPRRTMotor;

  BOOL   bHostRotBufMode;

  //LSJ 추가 기능... for Socket failure free (2006.03.08)
  SOCKET	ArraySocket[32];
#ifndef WIN64
  SOCKET	Dummy2[32];
#endif
  DWORD		ArrayPID[32];
  HANDLE	ArraySerial[32];
#ifndef WIN64
  HANDLE	Dummy3[32];
#endif
  DWORD		ArraySerPID[32];

  //LSJ 추가 기능... for ethernet multi-host application (2007.11.29)
  BOOL		bMultiHost;
  INT32		dwSockLockCnt;
} GLOBAL_HANDLE, *PGLOBAL_HANDLE;
#pragma pack(pop)
  #ifdef __cplusplus
}
  #endif

  #if defined( _DRIVER ) // DLL Include +++++++++++++++++++++++++++
                         // // debug macros
                         //  #if defined(DBG) || defined(_DEBUG)
                         //    void dbgPrintf(CHAR szFormat, ...);
                         //    extern int vcuDebugLevel;
                         //
                         //    #define dprintf(_x_)      dbgPrintf _x_
                         //    #define dprintf1(_x_)     if (vcuDebugLevel >= 1) dbgPrintf _x_
                         //    #define dprintf2(_x_)     if (vcuDebugLevel >= 2) dbgPrintf _x_
                         //    #define dprintf3(_x_)     if (vcuDebugLevel >= 3) dbgPrintf _x_
                         //    #define dprintf4(_x_)     if (vcuDebugLevel >= 4) dbgPrintf _x_
                         //  #else
                         //    #define dprintf(_x_)
                         //    #define dprintf1(_x_)
                         //    #define dprintf2(_x_)
                         //    #define dprintf3(_x_)
                         //    #define dprintf4(_x_)
                         // #endif

    #define WAIT_2SEC 2000 // Mutex object wait period of 2 sec.
    #define WAIT_5SEC 5000 // Mutex object wait period of 5 sec.
    #define TIMEOUT_COUNT_TIME    500


    #include <CRTDBG.H>
    #include <devioctl.h>
    #include <assert.h>
    #include "mcstruct.h"
    #include "mioctl.h"
    #include "registry.h"
    #include "bintoken.h"
    #include "cmplc56k.h"
    #include "pmacerr.h"
#ifndef _PMACSERVER
    #include "dprrealt.h"
    #include "dprbkg.h"
    #include "bus.h"
    #include "serial.h"
    #include "dprotlib.h"
    #include "hostbinrot.h"
    #include "cmplclib.h"
    #include "download.h"
    #include "utildt.h"
    #include "lips.h"
    #include "macro.h"
    #include "dialog.h"
#endif


#pragma pack(push, 4)
// Global externals
typedef struct tagGlobalData
{
  DWORD         dwSize;
  DWORD         dwUserCount; // Count uf users

  REG_ACCESS    RegistryAccess[MAX_MOTION_DEVICES + 1]; // registry access global
  CONFIG_LOCATION cfg[MAX_MOTION_DEVICES];              // Configuration values from registry
  GLOBAL_HANDLE gh[MAX_MOTION_DEVICES + 1];             // driver access handle

  HMODULE       hLangModule;
#ifndef WIN64
  HMODULE	Dummy0;
#endif
  char USResponse[MAX_MOTION_DEVICES + 1][USRESPONSE_SIZE];// Unsolicited response buffer
  DWORD         usr_next_in[MAX_MOTION_DEVICES + 1];
  DWORD         usr_next_out[MAX_MOTION_DEVICES + 1];

  char ChangeMonitor[MAX_MOTION_DEVICES + 1][IVARCHANGEMONITOR_SIZE];// Used for monitoring I-Variable Mod's by apps
  DWORD         cm_next_in[MAX_MOTION_DEVICES + 1];
  DWORD         cm_next_out[MAX_MOTION_DEVICES + 1];

  HANDLE        hServerPID ;
#ifndef WIN64
  HANDLE	Dummy1;
#endif
  DWORD         dwServerPID;
  BOOL          My_g_bDPRThreadRunning;
} GLOBAL_DATA, *PGLOBAL_DATA;
#pragma pack(pop) 

extern USER_HANDLE   vh[MAX_MOTION_DEVICES + 1 ]; // driver access handle
extern PGLOBAL_DATA  pGlobalData;
extern OSVERSIONINFO OsInfo;                      // Operating system information

  #ifdef __cplusplus
extern "C" {
  #endif


  #ifdef __cplusplus
}
  #endif

  #else // Application include +++++++++++++++++++++++++++++++++++++++++++++++++

    #include "mioctl.h"
    #include <mmsystem.h>
    #include "common.h"
    #include "bus.h"
    #include "dpr.h"
    #include "serial.h"
    #include "intr.h"
    #include "dprotlib.h"
    #include "hostbinrot.h"
    #include "dprbkg.h"
    #include "cmplclib.h"
    #include "pmacerr.h"
    #include "download.h"
    #include "utildt.h"
    #include "gather.h"
    #include "lips.h"
    #include "macro.h"

  #ifdef __cplusplus
extern "C" {
  #endif

extern HANDLE *ghLangModule;
extern USER_HANDLE   vh[MAX_MOTION_DEVICES + 1 ]; // driver access handle
extern GLOBAL_HANDLE gh[MAX_MOTION_DEVICES + 1 ]; // device access handle

  #ifdef __cplusplus
}
  #endif

  #endif


#endif //_PMACU_H_
