#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
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
