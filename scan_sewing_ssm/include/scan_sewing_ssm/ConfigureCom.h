#if !defined(AFX_CONFIGURECOM_H__2C6E95AC_D738_48ED_8DFA_C7938A480590__INCLUDED_)
#define AFX_CONFIGURECOM_H__2C6E95AC_D738_48ED_8DFA_C7938A480590__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ConfigureCom.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// ConfigureCom dialog

class CConfigureCom : public CDialog
{
// Construction
public:
	BOOL WriteRegistry();
	BOOL ReadRegistry();
	void SetDeviceNumber(DWORD dwDevice);
	DWORD m_dwDevice;
  DWORD m_dwEnum;
  DWORD m_dwBaudrate;
	CConfigureCom(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CConfigureCom)
	enum { IDD = IDD_CONFIGURE_COM };
	CComboBox	m_baudCB;
	CComboBox	m_parityCB;
	CComboBox	m_portCB;
	int		m_parity;
	int		m_port;
	CString	m_szDevice;
	UINT	m_charTimeout;
	UINT	m_flushTimeout;
	int		m_RTS;
	BOOL	m_bUseCTS;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CConfigureCom)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CConfigureCom)
	afx_msg void OnDeviceTest();
	virtual void OnOK();
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CONFIGURECOM_H__2C6E95AC_D738_48ED_8DFA_C7938A480590__INCLUDED_)
