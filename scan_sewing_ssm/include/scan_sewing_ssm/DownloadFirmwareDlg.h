#if !defined(AFX_DOWNLOADFIRMWAREDLG_H__134E7D33_70EA_4A01_8B3B_BBB016DF6890__INCLUDED_)
#define AFX_DOWNLOADFIRMWAREDLG_H__134E7D33_70EA_4A01_8B3B_BBB016DF6890__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DownloadFirmwareDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDownloadFirmwareDlg dialog

class CDownloadFirmwareDlg : public CDialog
{
// Construction
public:
	static void WINAPI DownloadMessage(LPTSTR msg,BOOL newline);
	static void WINAPI DownloadProgress(int percent);
	CString m_szFileName;
	DWORD m_dwEnum;
  LOCATIONTYPE m_loc;
	CDownloadFirmwareDlg(LOCATIONTYPE loc,DWORD dwEnum,CString filePath,CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CDownloadFirmwareDlg)
	enum { IDD = DLG_DOWNLOADFIRMWARE };
	CString	m_szPercentDone;
	CString	m_szMessage;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDownloadFirmwareDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	static CDownloadFirmwareDlg *pThis;

	// Generated message map functions
	//{{AFX_MSG(CDownloadFirmwareDlg)
	afx_msg void OnBegin();
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DOWNLOADFIRMWAREDLG_H__134E7D33_70EA_4A01_8B3B_BBB016DF6890__INCLUDED_)
