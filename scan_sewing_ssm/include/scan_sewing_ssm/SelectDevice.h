#if !defined(AFX_SELECTDEVICE_H__93C001F9_AB8F_4C03_AB0C_43F48260C427__INCLUDED_)
#define AFX_SELECTDEVICE_H__93C001F9_AB8F_4C03_AB0C_43F48260C427__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SelectDevice.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSelectDevice dialog

class CSelectDevice : public CDialog
{
// Construction
public:
	DWORD m_dwDevice;
	void UpdateDeviceList();
	CSelectDevice(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CSelectDevice)
	enum { IDD = IDD_SELECT_DEVICE };
	CButton	m_configureButton;
	CButton	m_testButton;
	CButton	m_okButton;
	CButton	m_insertButton;
	CButton	m_removeButton;
	CButton	m_userButton1;
	CButton	m_userButton2;
	CListBox	m_listBox;
	CString	m_szPmacDevice;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSelectDevice)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	BOOL LaunchUserApp(DWORD dwUser);
	LOCATIONTYPE m_locations[MAX_MOTION_DEVICES];
  bool         m_bInstalled[MAX_MOTION_DEVICES];

	// Generated message map functions
	//{{AFX_MSG(CSelectDevice)
	afx_msg void OnInsertButton();
	afx_msg void OnRemoveButton();
	virtual BOOL OnInitDialog();
	afx_msg void OnConfigureButton();
	afx_msg void OnDeviceTest();
	afx_msg void OnDblclkDeviceList();
	virtual void OnOK();
	afx_msg void OnSelchangeDeviceList();
	afx_msg void OnUser1();
	afx_msg void OnUser2();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SELECTDEVICE_H__93C001F9_AB8F_4C03_AB0C_43F48260C427__INCLUDED_)
