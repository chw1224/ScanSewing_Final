#if !defined(AFX_CONFIGUREISA_H__68591268_6DA4_43DC_890B_C15A84437C7E__INCLUDED_)
#define AFX_CONFIGUREISA_H__68591268_6DA4_43DC_890B_C15A84437C7E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ConfigureISA.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CConfigureISA dialog

class CConfigureISA : public CDialog
{
// Construction
public:
	BOOL ReadRegistry();
	BOOL WriteRegistry();
	void SetDeviceNumber(DWORD dwDevice);
	DWORD m_dwDevice;
  DWORD m_portAddress;
  DWORD m_dpramAddress;
  DWORD m_interruptNumber;
	CConfigureISA(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CConfigureISA)
	enum { IDD = IDD_CONFIGURE_ISA };
	CComboBox	m_intrCB;
	CComboBox	m_dpramCB;
	CComboBox	m_portCB;
	CString	m_szDevice;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CConfigureISA)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CConfigureISA)
	afx_msg void OnDeviceTest();
	virtual void OnOK();
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CONFIGUREISA_H__68591268_6DA4_43DC_890B_C15A84437C7E__INCLUDED_)
