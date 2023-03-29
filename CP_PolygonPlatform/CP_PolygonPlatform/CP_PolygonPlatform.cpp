// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问  
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// CP_PolygonPlatform.cpp : 定义应用程序的类行为。
//


#include "stdafx.h"
#include "afxwinappex.h"
#include "afxdialogex.h"
#include "CP_PolygonPlatform.h"
#include "MainFrm.h"

#include "CP_PolygonPlatformDoc.h"
#include "CP_PolygonPlatformView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CCP_PolygonPlatformApp

BEGIN_MESSAGE_MAP(CCP_PolygonPlatformApp, CWinAppEx)
	ON_COMMAND(ID_APP_ABOUT, &CCP_PolygonPlatformApp::OnAppAbout)
	// 샘黨匡숭돨깃硫匡도츱즈
	ON_COMMAND(ID_FILE_NEW, &CWinAppEx::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinAppEx::OnFileOpen)
	// 깃硫댔丹零츱즈
	ON_COMMAND(ID_FILE_PRINT_SETUP, &CWinAppEx::OnFilePrintSetup)
END_MESSAGE_MAP()


// CCP_PolygonPlatformApp 뭐芚

CCP_PolygonPlatformApp::CCP_PolygonPlatformApp()
{
	// 連넣路劤폘땡밗잿포
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_ALL_ASPECTS;
#ifdef _MANAGED
	// 흔벎壇痰넋埼角적痰무묾刀喇頓契珂連넣(/clr)뭐쉔돨，橙:
	//     1) 극伎唐늪맒속零，“路劤폘땡밗잿포”連넣꼽콘攣끽묏鱗。
	//     2) 瞳퀭돨淃커櫓，퀭극伎객亮냥糠埼蕨 System.Windows.Forms 警속多痰。
	System::Windows::Forms::Application::SetUnhandledExceptionMode(System::Windows::Forms::UnhandledExceptionMode::ThrowException);
#endif

	// TODO: 쉥鹿苟壇痰넋埼 ID 俚륜눔競뻣槨顆寧돨 ID 俚륜눔；쉔累돨俚륜눔목駕
	//槨 CompanyName.ProductName.SubProduct.VersionInformation
	SetAppID(_T("CP_PolygonPlatform.AppID.NoVersion"));

	// TODO: 瞳늪뇹警속뭐芚덜쯤，
	// 쉥杰唐路狼돨놓迦뺏렴零瞳 InitInstance 櫓
}

// 顆寧돨寧몸 CCP_PolygonPlatformApp 뚤蹶

CCP_PolygonPlatformApp theApp;


// CCP_PolygonPlatformApp 놓迦뺏

BOOL CCP_PolygonPlatformApp::InitInstance()
{
	// 흔벎寧몸頓契瞳 Windows XP 돨壇痰넋埼헌데寧땍狼
	// 賈痰 ComCtl32.dll 경굶 6 샀뫘멕경굶윱폘痰옵柬뺏렘駕，
	//橙矜狼 InitCommonControlsEx()。뤠橙，쉥轟랬눼쉔눗왯。
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 쉥劍零槨관윅杰唐狼瞳壇痰넋埼櫓賈痰돨
	// 무묾왠숭잚。
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinAppEx::InitInstance();


	// 놓迦뺏 OLE 욋
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}

	AfxEnableControlContainer();

	EnableTaskbarInteraction(FALSE);

	// 賈痰 RichEdit 왠숭矜狼  AfxInitRichEdit2()	
	// AfxInitRichEdit2();

	// 깃硫놓迦뺏
	// 흔벎灌賈痰侶硅묘콘깻句寡숑鬼
	// 離老옵獵契匡숭돨댕鬼，橙壇盧뇜苟죗
	// 꼇矜狼돨景땍놓迦뺏절넋
	// 뫘맣痰黨닸뇨零돨鬧꿍깊淃
	// TODO: 壇刊뎠錦맣맡俚륜눔，
	// 절흔錦맣槨무鱇샀莉廉츰
	SetRegistryKey(_T("壇痰넋埼蕨돔냥돨굶뒈壇痰넋埼"));
	LoadStdProfileSettings(4);  // 속潼깃硫 INI 匡숭朞淃(관윅 MRU)


	InitContextMenuManager();

	InitKeyboardManager();

	InitTooltipManager();
	CMFCToolTipInfo ttParams;
	ttParams.m_bVislManagerTheme = TRUE;
	theApp.GetTooltipManager()->SetTooltipParams(AFX_TOOLTIP_TYPE_ALL,
		RUNTIME_CLASS(CMFCToolTipCtrl), &ttParams);

	// 鬧꿍壇痰넋埼돨匡도친겼。匡도친겼
	// 쉥痰鱗匡도、움솥눗왯뵨柬暠裂쇌돨젯쌈
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CCP_PolygonPlatformDoc),
		RUNTIME_CLASS(CMainFrame),       // 寮 SDI 움솥눗왯
		RUNTIME_CLASS(CCP_PolygonPlatformView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);


	// 롸驕깃硫 shell 츱즈、DDE、댔역匡숭꾸鱗돨츱즈契
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);

	// 폘痰“DDE 獵契”
	EnableShellOpen();
	RegisterShellFileTypes(TRUE);


	// 딧똑瞳츱즈契櫓寧땍돨츱즈。흔벎
	// 痰 /RegServer、/Register、/Unregserver 샀 /Unregister 폘땡壇痰넋埼，橙럿쀼 FALSE。
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// 顆寧돨寧몸눗왯綠놓迦뺏，凜늪鞫刻劍깻뚤페쏵契뫘劤
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	// 쏭뎠야唐빈留珂꼽딧痰 DragAcceptFiles
	//  瞳 SDI 壇痰넋埼櫓，侶壇瞳 ProcessShellCommand 裂빈랙
	// 폘痰賈/렴
	m_pMainWnd->DragAcceptFiles();
	return TRUE;
}

int CCP_PolygonPlatformApp::ExitInstance()
{
	//TODO: 뇹잿옵콘綠警속돨맒속栗都
	AfxOleTerm(FALSE);

	return CWinAppEx::ExitInstance();
}

// CCP_PolygonPlatformApp 句口뇹잿넋埼


// 痰黨壇痰넋埼“밑黨”꽉데淃돨 CAboutDlg 뚤뺐움

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 뚤뺐움鑒앴
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 連넣

// 茄君
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// 痰黨頓契뚤뺐움돨壇痰넋埼츱즈
void CCP_PolygonPlatformApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

// CCP_PolygonPlatformApp 菱땍屢속潼/괏닸렘랬

void CCP_PolygonPlatformApp::PreLoadState()
{
	BOOL bNameValid;
	CString strName;
	bNameValid = strName.LoadString(IDS_EDIT_MENU);
	ASSERT(bNameValid);
	GetContextMenuManager()->AddMenu(strName, IDR_POPUP_EDIT);
}

void CCP_PolygonPlatformApp::LoadCustomState()
{
}

void CCP_PolygonPlatformApp::SaveCustomState()
{
}

// CCP_PolygonPlatformApp 句口뇹잿넋埼



