#if !defined(AFX_SELECTCOMMTYPE_H__F455F475_0211_470B_A94D_B9D8F6974ED6__INCLUDED_)
#define AFX_SELECTCOMMTYPE_H__F455F475_0211_470B_A94D_B9D8F6974ED6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SelectCommType.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSelectCommType dialog

class CSelectCommType : public CDialog
{
// Construction
public:
	CSelectCommType(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CSelectCommType)
	enum { IDD = IDD_SELECT_COMM_TYPE };
	int		m_pmacCommType;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSelectCommType)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CSelectCommType)
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SELECTCOMMTYPE_H__F455F475_0211_470B_A94D_B9D8F6974ED6__INCLUDED_)
