/*
HISTORY

09Jul99 JET Added DrvNextValue() and DrvCloseValue() definitions

/****************************************************************************
 *
 *   registry.h
 *
 *   Copyright (c) 1992 Microsoft Corporation.  All Rights Reserved.
 *
 *   This file contains public definitions for maintaining registry information
 *   for drivers managing kernel driver registry related data.
 ****************************************************************************/

#ifndef _REGISTRY_H
  #define _REGISTRY_H

#include <winsvc.h>

/***************************************************************************
 *
 * Constants for accessing the registry
 *
 ***************************************************************************/

/*
 *  Path to service node key
 */

#define STR_SERVICES_NODE TEXT("SYSTEM\\CurrentControlSet\\Services\\")

/*
 *  Node sub-key for device parameters
 */

//#define STR_DEVICE_DATA TEXT("Parameters")

/*
 *  Name of Base group where sound drivers normally go
 */

#define STR_BASE_GROUP TEXT("Base")

/*
 *  Name of driver group for synthesizers
 *     - we use our own name here to make sure
 *       we are loaded after things like the PAS driver.  (Base is a
 *       known group and drivers in it are always loaded before unknown
 *       groups like this one).
 */

#define STR_SYNTH_GROUP TEXT("Motion Drivers")

/*
 *  Name of service
 */

#define STR_DRIVER TEXT("\\Driver\\")

/*
 *  Path to kernel drivers directory from system directory
 */

#define STR_DRIVERS_DIR TEXT("\\SystemRoot\\System32\\drivers\\")

/*
 *  Extension for drivers
 */

#define STR_SYS_EXT TEXT(".SYS")

/****************************************************************************

 Driver types

 ****************************************************************************/

typedef enum {
    SoundDriverTypeNormal = 1,
    SoundDriverTypeSynth /* Go in the synth group */
} SOUND_KERNEL_MODE_DRIVER_TYPE;

/****************************************************************************

Our registry access data

****************************************************************************/
#pragma pack(push, 4)
typedef struct {
    SC_HANDLE ServiceManagerHandle;           // Handle to the service controller
#ifndef WIN64
  SC_HANDLE	Dummy0;
#endif
    CHAR  DriverName[20];                    // Name of driver
    CHAR  TempKeySaveFileName[MAX_PATH];     // Where parameters key is saved
    DWORD  dwDevice;                          // Number current device
    SOUND_KERNEL_MODE_DRIVER_TYPE DriverType; // Type of device driver
    DWORD  dwDeviceUsage;                     // How many users this device
} REG_ACCESS, *PREG_ACCESS;
#pragma pack(pop)

/****************************************************************************

Test if configuration etc can be supported

****************************************************************************/

  #define DrvAccess(RegAccess) ((RegAccess)->ServiceManagerHandle != NULL)

/****************************************************************************

 Function prototypes

 ****************************************************************************/
  #ifdef __cplusplus
extern "C" {
  #endif

  //HKEY CALLBACK DrvCreateDeviceKey(PREG_ACCESS RegAccess);
  HKEY CALLBACK DrvOpenRegKey( DWORD dwDevice, LPCTSTR Path );
  BOOL CALLBACK DrvSetKnownVxd( DWORD dwDevice );
  BOOL CALLBACK DrvCreateServicesNode( DWORD dwDevice, BOOL Create );
  VOID CALLBACK DrvCloseServiceManager( DWORD dwDevice );
  BOOL CALLBACK DrvDeleteServicesNode( DWORD dwDevice );
  //BOOL CALLBACK DrvSaveParametersKey(PREG_ACCESS RegAccess);
  //BOOL CALLBACK DrvRestoreParametersKey(PREG_ACCESS RegAccess);
  HKEY CALLBACK DrvOpenDeviceKey( DWORD dwDevice );

  //      Set a device DWORD parameter
  LONG CALLBACK DrvSetDeviceDword(
    DWORD dwDevice,
    LPTSTR ValueName,
    DWORD dwValue );

  //     Read current DWORD parameter setting
  LONG CALLBACK DrvQueryDeviceDword(
    DWORD dwDevice,
    PTCHAR ValueName,
    PDWORD pValue,
    DWORD  defValue );

  //      Set a device STRING parameter
  LONG CALLBACK DrvSetDeviceString(
    DWORD dwDevice,
    PTCHAR ValueName,
    PTCHAR Value );

  //     Read current STRING parameter setting
  LONG CALLBACK DrvQueryDeviceString(
    DWORD dwDevice,
    PTCHAR ValueName,
    PTCHAR pValue,
    DWORD  valuelength,
    PTCHAR defValue );

  BOOL CALLBACK DrvLoadKernelDriver( DWORD dwDevice );
  BOOL CALLBACK DrvUnloadKernelDriver( DWORD dwDevice );
  BOOL CALLBACK DrvIsDriverLoaded( DWORD dwDevice );
  BOOL CALLBACK DrvConfigureDriver( DWORD dwDevice,PVOID Context );

  LRESULT CALLBACK DrvRemoveDriver( DWORD dwDevice );
  BOOL CALLBACK DrvRemoveDevice( DWORD dwDevice );
  LONG CALLBACK DrvNumberOfDevices();
  VOID CALLBACK DrvSetMapperName( LPTSTR SetupName );

  // Nc Specific
  HKEY CALLBACK DrvOpenNcKey( DWORD device, DWORD control, LPCTSTR Section );
  LONG CALLBACK DrvSetNcDword( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, DWORD Value );
  LONG CALLBACK DrvQueryNcDword( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, PDWORD pValue,
                   DWORD defValue );
  LONG CALLBACK DrvSetNcDouble( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, double Value );
  LONG CALLBACK DrvQueryNcDouble( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, double *pValue,
                   double defValue );
  LONG CALLBACK DrvSetNcString( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, LPTSTR Value );
  LONG CALLBACK DrvQueryNcString( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, LPTSTR pValue,
                   DWORD ValueLength, LPTSTR defValue );
  LONG CALLBACK DrvSetNcBool( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, BOOL Value );
  LONG CALLBACK DrvQueryNcBool( DWORD device, DWORD ControlNumber, LPCTSTR Section, LPTSTR ValueName, PBOOL pValue,
                   BOOL defValue );
  void          DrvCloseValue(DWORD device, DWORD control);
  BOOL          DrvNextValue(DWORD device,DWORD control,LPCTSTR Reg_Key,LPTSTR value_name_bfr);

  #ifdef __cplusplus
}
  #endif

#endif // _REGISTRY_H

