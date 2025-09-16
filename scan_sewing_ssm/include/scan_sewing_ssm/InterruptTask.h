#if !defined(AFX_INTERRUPTTASK_H__87611119_3604_461A_B23C_32324FF7C553__INCLUDED_)
#define AFX_INTERRUPTTASK_H__87611119_3604_461A_B23C_32324FF7C553__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// InterruptTask.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CInterruptTask thread

class CInterruptTask
{
public:
	CInterruptTask(BYTE mask = 0, DWORD task = 0,DWORD dwUser = 0,HANDLE hEvent = NULL, 
    HWND hWnd = NULL, PMACINTRPROC pCallback = NULL,
    LPTHREAD_START_ROUTINE  pThreadFunc = NULL);
	virtual ~CInterruptTask();

  bool DoTask(DWORD dwEvent);

  BYTE    m_intrMask;       // Interrupt mask (IPOS - IR7)
  DWORD   m_taskMask;       // What to do 
                            // (CALLBACK_WINDOW,CALLBACK_FUNCTION,CALLBACK_THREAD)
  DWORD   m_dwUser;         // User information
  HANDLE  m_hEvent;         // Event handle for event task
  HWND    m_hWnd;           // Window handle for sendmessage task
  PMACINTRPROC m_pCallback;
  LPTHREAD_START_ROUTINE  m_pThreadFunc;

  INTRBUFFER m_buffer;
};

/////////////////////////////////////////////////////////////////////////////

extern CList<CInterruptTask, CInterruptTask&> interruptTaskList;

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_INTERRUPTTASK_H__87611119_3604_461A_B23C_32324FF7C553__INCLUDED_)
