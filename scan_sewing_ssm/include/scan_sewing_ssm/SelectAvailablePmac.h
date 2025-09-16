#if !defined(AFX_SELECTAVAILABLEPMAC_H__78E7B65C_03D5_47D0_83B5_B02881EB1438__INCLUDED_)
#define AFX_SELECTAVAILABLEPMAC_H__78E7B65C_03D5_47D0_83B5_B02881EB1438__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SelectAvailablePmac.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSelectAvailablePmac dialog

class CSelectAvailablePmac : public CDialog
{
// Construction
public:
	DWORD m_dwPort;
	LOCATIONTYPE m_pmacCommType;
	BOOL SetDeviceNumber(DWORD dwDevice);
	DWORD m_dwDevice;
	CSelectAvailablePmac(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CSelectAvailablePmac)
	enum { IDD = IDD_AVAILABLE_DEVICES };
	CListBox	m_pmacList;
	CString	m_szPmacDesc;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSelectAvailablePmac)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	UINT m_nTimer;
	void UpdateDeviceList();

	// Generated message map functions
	//{{AFX_MSG(CSelectAvailablePmac)
	afx_msg void OnNewButton();
	afx_msg void OnTimer(UINT nIDEvent);
	virtual BOOL OnInitDialog();
	afx_msg void OnShowWindow(BOOL bShow, UINT nStatus);
	afx_msg void OnDblclkPmacList();
	virtual void OnOK();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SELECTAVAILABLEPMAC_H__78E7B65C_03D5_47D0_83B5_B02881EB1438__INCLUDED_)
