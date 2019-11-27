#pragma once


// CLanSetupDlg ダイアログ

class CLanSetupDlg : public CDialog
{
	DECLARE_DYNAMIC(CLanSetupDlg)

public:
	CLanSetupDlg(CWnd* pParent = NULL);   // 標準コンストラクタ
	virtual ~CLanSetupDlg();

// ダイアログ データ
	enum { IDD = IDD_DIALOG_LANSETUP };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV サポート

	DECLARE_MESSAGE_MAP()
public:
	CString m_strHandle;
public:
	CString m_strServer;
public:
	UINT m_nChannel;
public:
	UINT m_nLevel;
public:
	BOOL m_bCheckLog;
};
