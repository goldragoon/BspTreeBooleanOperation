#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"

class CCP_PolygonPlatformApp : public CWinAppEx
{
public:
	CCP_PolygonPlatformApp();

public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

	UINT  m_nAppLook;
	virtual void PreLoadState();
	virtual void LoadCustomState();
	virtual void SaveCustomState();

	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CCP_PolygonPlatformApp theApp;
