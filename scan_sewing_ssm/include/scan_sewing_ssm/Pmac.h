// Pmac.h : main header file for the PMAC DLL
//

#if !defined(AFX_PMAC_H__F9437814_080F_4362_9C63_7FC2518B1479__INCLUDED_)
#define AFX_PMAC_H__F9437814_080F_4362_9C63_7FC2518B1479__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols
#include "DOWNLOAD.H"	// Added by ClassView
#include "pmacserver.h"

/////////////////////////////////////////////////////////////////////////////
// CPmacApp
// See Pmac.cpp for the implementation of this class
//

class CPmacApp : public CWinApp
{
public:
	BOOL OnLine(DWORD dwDevice);
	BOOL OnLine(LOCATIONTYPE loc, DWORD dwEnum);
	DWORD m_dwUserCount[MAX_MOTION_DEVICES];
  BOOL ReferenceInterrupts(LOCATIONTYPE loc,DWORD dwEnum);
	BOOL InitDeviceSerialPort(DWORD dwDevice);
	BOOL SetAsciiComm(LOCATIONTYPE loc,DWORD dwEnum,ASCIIMODE mode);
  DWORD MapDeviceNumToEnum(LOCATIONTYPE loc,DWORD dwEnum);
  DWORD MapLocAndEnumFromDeviceNum(DWORD dwDevNum, LOCATIONTYPE *loc,DWORD *dwEnum);

	BOOL DownloadFirmware(DWORD dwDevice);
	void UnRegisterDevice(DWORD dwDevice);
	void RegisterDevice(DWORD dwDevice,LOCATIONTYPE type,DWORD port);
	BOOL RegisteredDevice(LOCATIONTYPE type,DWORD port,PDWORD pdwDevice);
	BOOL ConfigureDevice(DWORD dwDevice,LOCATIONTYPE lt);
  BOOL SetDeviceDword(DWORD dwDevice,CString KeyName,DWORD Value);
  BOOL GetDeviceDword(DWORD dwDevice,CString KeyName,PDWORD pValue,DWORD defValue);
  BOOL SetDeviceString(DWORD dwDevice,CString KeyName,CString Value);
  BOOL GetDeviceString(DWORD dwDevice,CString KeyName,CString &Value,CString defValue);
  BOOL GetRegistryString(CString KeyName,CString &Value,CString defValue);
  BOOL SetRegistryString(CString KeyName,CString Value);
  BOOL SetDevicePmacDword(DWORD dwDevice,CString KeyName,DWORD dwValue);
  BOOL GetDevicePmacDword(DWORD dwDevice,CString KeyName,PDWORD pValue,DWORD defValue);
  VOID GetDevicePmacParameters(DWORD dwDevice, PCONFIG_LOCATION pLoc);
  BOOL SetDevicePmacParameters(DWORD dwDevice,CONFIG_LOCATION cfg);
	BOOL RemoveDeviceKey(DWORD dwDevice);
	BOOL IsDeviceKeyAvailable(DWORD dwDevice);
  HKEY OpenDeviceKey(DWORD dwDevice,BOOL bCreate);
  HKEY OpenRegKey(LPCTSTR Path,BOOL bCreate);

  BOOL OpenPmacDevice(DWORD dwDevice);
  BOOL ClosePmacDevice(DWORD dwDevice);
  BOOL ShutdownPmacDevice(DWORD dwDevice,BOOL bForceShutdown);

  BOOL GetCommState(DWORD dwDevice);
  BOOL SetCommState(DWORD dwDevice);
  BOOL TransmitCommChar(DWORD dwDevice,TCHAR ch);
  BOOL PurgeComm(DWORD dwDevice);

	void ClosePmac(LOCATIONTYPE loc,DWORD dwEnum);
	BOOL OpenPmac(LOCATIONTYPE loc,DWORD dwEnum);
  HKEY OpenPmacKey(LOCATIONTYPE loc,DWORD dwEnum,BOOL bCreate);
	BOOL IsPmacKeyAvailable(LOCATIONTYPE loc,DWORD dwEnum);
  BOOL SetPmacDword(LOCATIONTYPE loc,DWORD dwEnum,CString KeyName,DWORD Value);
  BOOL GetPmacDword(LOCATIONTYPE loc,DWORD dwEnum,CString KeyName,PDWORD pValue,DWORD defValue);
  BOOL SetPmacString(LOCATIONTYPE loc,DWORD dwEnum,CString KeyName,CString Value);
  BOOL GetPmacString(LOCATIONTYPE loc,DWORD dwEnum,CString KeyName,CString &Value,CString defValue);
  BOOL DetectDevice(DWORD dwDevice,PDWORD pdwError);
  BOOL DetectPmac(LOCATIONTYPE loc,DWORD dwEnum,PDWORD pdwError);
  VOID GetPmacParameters(LOCATIONTYPE loc, DWORD dwEnum, PCONFIG_LOCATION pCfg );
  BOOL SetPmacParameters(LOCATIONTYPE loc, DWORD dwEnum, CONFIG_LOCATION cfg);
	BOOL DownloadFirmware(LOCATIONTYPE loc, DWORD dwEnum);
	BOOL IsPmacInBootstrap(DWORD dwDevice,LOCATIONTYPE loc,DWORD dwEnum);

  int  DownloadFirmwareFile(LOCATIONTYPE loc,DWORD dwEnum, DOWNLOADMSGPROC msgp,
                                       DOWNLOADPROGRESS prgp, LPCTSTR filename );
  static void _cdecl FirmwareThread( PVOID pVoid );
  static void _cdecl FirmwareThreadUSB( PVOID pVoid );
  static void _cdecl FirmwareThreadETH( PVOID pVoid );

  // These functions are for kernel driver access using the Service Control
  // Manager of NT / 2000. PNP does not require these only our legacy ISA
  BOOL SC_CreateNode(LPCTSTR DriverName, BOOL bCreate );
  BOOL SC_LoadKernelDriver(LPCTSTR DriverName);
  BOOL SC_UnloadKernelDriver(LPCTSTR DriverName);
  BOOL SC_IsDriverLoaded(LPCTSTR DriverName);
  BOOL SC_ConfigureDriver(LOCATIONTYPE loc,DWORD dwEnum,PVOID Context);

  CPmacApp();

  DOWNLOAD       m_download;
  int            m_lastError;
  DWORD          m_dwDevice;
  HANDLE         m_hDriver[MAX_PMAC_LOCATIONS][MAX_MOTION_DEVICES];
  DEVICEINTARRAY m_Interrupts[MAX_MOTION_DEVICES];
  IPmacService   *m_pServer;
  HANDLE         hSourceProcess;

  // Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPmacApp)
	public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();
	virtual BOOL InitApplication();
	//}}AFX_VIRTUAL

	//{{AFX_MSG(CPmacApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

extern CPmacApp theApp;

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PMAC_H__F9437814_080F_4362_9C63_7FC2518B1479__INCLUDED_)
