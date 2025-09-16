#if !defined(AFX_CONFIGUREDPR_H__CA2E409F_B42A_4013_ADD8_7BC7CDF7C60A__INCLUDED_)
#define AFX_CONFIGUREDPR_H__CA2E409F_B42A_4013_ADD8_7BC7CDF7C60A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ConfigureDPR.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CConfigureDPR dialog

class CConfigureDPR : public CDialog
{
// Construction
public:
	CConfigureDPR(CWnd* pParent = NULL);   // standard constructor
  BOOL WriteRegistry();
	BOOL ReadRegistry();
	void SetDeviceNumber(DWORD dwDevice);
	DWORD m_dwDevice;
  DWORD m_dwEnum;
  LOCATIONTYPE m_dwLoc;
  int m_DprRealTimeMotorMask;

// Dialog Data
	//{{AFX_DATA(CConfigureDPR)
	enum { IDD = IDD_CONFIGURE_DPR };
	CButton	m_Motor8MaskCtrl;
	CButton	m_Motor7MaskCtrl;
	CButton	m_Motor6MaskCtrl;
	CButton	m_Motor5MaskCtrl;
	CButton	m_Motor4MaskCtrl;
	CButton	m_Motor3MaskCtrl;
	CButton	m_Motor2MaskCtrl;
	CButton	m_Motor1MaskCtrl;
	CButton	m_DprRealTimeUpdateCtrl;
	CButton	m_DprBkgUpdateCtrl;
	CButton	m_DprAutoUpdateCtrl;
  int m_DprAutoUpdate;
  int m_DprBkgUpdate;
  int m_DprRealTimeUpdate;
  int m_DprBkgPeriod;
  int m_DprRealTimePeriod;
  int m_EnableInterrupts;
  int m_DprMonitorPeriod;
	//}}AFX_DATA
	CString	m_szDevice;


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CConfigureDPR)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
  int CheckMotorMask();
  void DisplayMotorMask(int mask);
	// Generated message map functions
	//{{AFX_MSG(CConfigureDPR)
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	afx_msg void OnDprautoupdate();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CONFIGUREDPR_H__CA2E409F_B42A_4013_ADD8_7BC7CDF7C60A__INCLUDED_)
