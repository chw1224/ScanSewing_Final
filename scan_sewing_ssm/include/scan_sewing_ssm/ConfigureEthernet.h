#if !defined(AFX_CONFIGUREETHERNET_H__369AF88D_687E_4280_8A26_9CE802898DFC__INCLUDED_)
#define AFX_CONFIGUREETHERNET_H__369AF88D_687E_4280_8A26_9CE802898DFC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ConfigureEthernet.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CConfigureEthernet dialog

class CConfigureEthernet : public CDialog
{
// Construction
public:
	BOOL  WriteRegistry(BOOL bTest);
	DWORD ReadRegistry();
	void SetDeviceNumber(DWORD dwDevice);
	DWORD        m_dwDevice;
  DWORD        m_dwEnum;
  LOCATIONTYPE m_dwLoc;
  DWORD        m_originalIP;
	CConfigureEthernet(CWnd* pParent = NULL);   // standard constructor
	int	m_DprRealTimeMotorMask;

// Dialog Data
	//{{AFX_DATA(CConfigureEthernet)
	enum { IDD = IDD_CONFIGURE_ETH };
	CButton	m_Motor8MaskCtrl;
	CButton	m_Motor7MaskCtrl;
	CButton	m_Motor6MaskCtrl;
	CButton	m_Motor5MaskCtrl;
	CButton	m_Motor4MaskCtrl;
	CButton	m_Motor3MaskCtrl;
	CButton	m_Motor2MaskCtrl;
	CButton	m_Motor1MaskCtrl;
	CButton	m_DprBkgUpdateCtrl;
	CButton	m_DprRealTimeUpdateCtrl;
	CButton	m_DprAutoUpdateCtrl;
	CIPAddressCtrl	m_ipAddress;
	CString	m_szDevice;
	int	m_DprAutoUpdate;
	int	m_DprBkgUpdate;
	int	m_DprRealTimeUpdate;
  int	m_DprBkgPeriod;
	int	m_DprRealTimePeriod;
	//}}AFX_DATA

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CConfigureEthernet)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

  int CheckMotorMask();
  void DisplayMotorMask(int mask);
	// Generated message map functions
	//{{AFX_MSG(CConfigureEthernet)
	afx_msg void OnDeviceTest();
	virtual void OnOK();
	virtual BOOL OnInitDialog();
	afx_msg void OnDprautoupdate();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CONFIGUREETHERNET_H__369AF88D_687E_4280_8A26_9CE802898DFC__INCLUDED_)
