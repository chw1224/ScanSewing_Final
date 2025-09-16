/*
 * dialog.h
 *
 * 32-bit Motion Control Device Driver
 *
 */

  #ifdef __cplusplus
extern "C" {
  #endif

typedef struct tag_combobox_entry {
   DWORD wValue;
   LPTSTR szText;
} COMBOBOX_ENTRY;

LRESULT ConfigRemove(DWORD dwDevice, HWND hDlg );
int     Config( HWND hWnd, PREG_ACCESS pRegAccess );
VOID    ConfigErrorMsgBox( HWND hDlg, DWORD wStringId );
int     ConfigLoadComboBox( HWND hDlg, int wID, COMBOBOX_ENTRY *pCBE, int nEntries, DWORD wInitialValue );
DWORD   ConfigGetComboBoxValue( HWND hDlg, int wID, COMBOBOX_ENTRY *pCBE );
int     ConfigSetComboBoxSelection( HWND hDlg, int wID, COMBOBOX_ENTRY *pCBE, int nEntries, DWORD dwSelectValue );
DWORD   ConfigGetEditValue( HWND hDlg, int wID, int numberbase );

DWORD   ConfigConfirmLegalValue( COMBOBOX_ENTRY *pCBE, int nEntries, DWORD wValueToTest );

long CALLBACK    PmacSelect( HWND hWnd );

  #ifdef __cplusplus
}
  #endif

