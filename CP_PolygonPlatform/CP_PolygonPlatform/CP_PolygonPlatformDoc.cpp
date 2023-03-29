﻿// 侶뙈 MFC 刻절都덜쯤蘿刻흔부賈痰 MFC Microsoft Office Fluent 痰빵썹충 
// (“Fluent UI”)。맡刻절쏭묩꽝옘，
// 痰鹿껸념《Microsoft 샘뇟잚꽝옘》뵨 
// MFC C++ 욋흡숭踞맒돨宮밑든綾匡도。
// 릿齡、賈痰샀롸랙 Fluent UI 돨冀옵係운角데뗌瓊묩돨。
// 흼狼죄썩唐밑 Fluent UI 冀옵셕뺍돨圈玖斤口，헝련狂  
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 경홈杰唐(C) Microsoft Corporation
// 괏즛杰唐홈적。

// CP_PolygonPlatformDoc.cpp : CCP_PolygonPlatformDoc 잚돨茄君
//

#include "stdafx.h"
// SHARED_HANDLERS 옵鹿瞳茄君渡응、鍵쫠暠뵨鎧乞朞포얌깨돨
// ATL 淃커櫓쏵契땍屢，깻豚冀宅맡淃커묾權匡도덜쯤。
#ifndef SHARED_HANDLERS
#include "CP_PolygonPlatform.h"
#endif

#include "CP_PolygonPlatformDoc.h"

#ifdef __INTELLISENSE__
#include <afxole.h>
#include <afxtabctrl.h>
#include <afxmdichildwndex.h>
#include <afxoutlookbartabctrl.h>
#include <afxpanecontainermanager.h>
#endif

#include <propkey.h>
#include <sstream>
#include <string>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CCP_PolygonPlatformDoc

IMPLEMENT_DYNCREATE(CCP_PolygonPlatformDoc, CDocument)

BEGIN_MESSAGE_MAP(CCP_PolygonPlatformDoc, CDocument)
END_MESSAGE_MAP()


// CCP_PolygonPlatformDoc 뭐芚/驕뭐

CCP_PolygonPlatformDoc::CCP_PolygonPlatformDoc()
{
	// TODO: 瞳늪警속寧늴昑뭐芚덜쯤
	mb_initData( );
}

CCP_PolygonPlatformDoc::~CCP_PolygonPlatformDoc()
{
}

void CCP_PolygonPlatformDoc::mb_initData( )
{
    m_a.mb_clear( );
    m_b.mb_clear( );
    m_tolerance = 1e-6; // 貫零휭뀌
    m_scale = 1.0; // 鍵렴궐절
    m_translation.m_x = 0.0; // 麟깃틱盧좆
    m_translation.m_y = 0.0; // 麟깃틱盧좆
    m_result.mb_clear( );
    m_flagBuildA = true; // true: A; false B。
    m_flagSelect = false; 
    m_flagSelectType = 0; 
    m_flagSelectPolygon = 0;
    m_flagSelectRegion = 0;
    m_flagSelectID = 0; // 땍貫歌혤돨코휭
	m_flagShowSelect = false; // true:怜鞫刻朞嶝섞。
    m_edgeNumber = 3; // 攣뜩긋近돨긋鑒。
    m_flagMouseDown = false; // true: 객苟柑깃璘숩
    m_flagAdd = 0;
    m_flagShowA = true;
    m_flagShowB = true;
    m_flagShowPointID = false;
    m_flagMoveSame = false;
    m_flagSelectIDSetInA.clear( );
    m_flagSelectIDSetInB.clear( );

	m_showBsptree = false;
	m_bspTree = NULL;
} // 잚CCP_PolygonPlatformDoc돨냥逃변鑒mb_initData써監

BOOL CCP_PolygonPlatformDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	mb_initData( );
	CString string;
    auto window = (CFrameWndEx*)(AfxGetApp()->GetMainWnd());

    CMFCRibbonBar* robbon_bar = window->GetRibbonBar(); //삿혤Ribbon bar 얌깨
    if (robbon_bar==NULL)
        return TRUE;
    CMFCRibbonEdit* slider = (CMFCRibbonEdit*)robbon_bar->FindByID(ID_TOLERANCE); // 삿혤긍서움얌깨
    if (slider==NULL)
        return TRUE;
    string.Format("%g", m_tolerance);
    slider->SetEditText(string);
    CMFCRibbonComboBox* pbox = (CMFCRibbonComboBox*)robbon_bar->FindByID(ID_COMBO_AorB); // 삿혤긍서움얌깨
    if (pbox==NULL)
        return TRUE;
    pbox->AddItem("뜩긋近A");
    pbox->AddItem("뜩긋近B");
    pbox->SelectItem(0);

	return TRUE;
}




// CCP_PolygonPlatformDoc 埼죗뺏

void CCP_PolygonPlatformDoc::Serialize(CArchive& ar)
{
	CString line;

	if (ar.IsStoring())
	{
		// TODO: 瞳늪警속닸뇨덜쯤
		ar.WriteString("# Polygon Data(啖에베: 셕炬샙릅燎섯부芚謹(헌빽댕欺, 2014헬))\r\n");
        ar.WriteString("# 匡숭눼쉔珂쇌: ");
        SYSTEMTIME st;
        CString strDate,strTime;
        GetLocalTime(&st);
        strDate.Format("%4d-%2d-%2d ",st.wYear,st.wMonth,st.wDay);
        strTime.Format("%2d:%2d:%2d\r\n\r\n",st.wHour,st.wMinute,st.wSecond);
        ar.WriteString(strDate.GetString());
        ar.WriteString(strTime.GetString());
		line.Format("Tolerance %g\r\n\r\n", m_tolerance);
        ar.WriteString(line.GetString());
        line.Format("Coordinate %g %g %g\r\n\r\n", m_scale, m_translation.m_x, m_translation.m_y);
        ar.WriteString(line.GetString());
        line.Format("A Polygon\r\n\r\n");
        ar.WriteString(line.GetString());
        gb_SerializePolygon(ar, m_a);
        line.Format("B Polygon\r\n\r\n");
        ar.WriteString(line.GetString());
        gb_SerializePolygon(ar, m_b);
	}
	else
	{
		// TODO: 瞳늪警속속潼덜쯤
		mb_initData( );
        while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'T')
                {
                    string temp;
                    stringstream ss(line.GetString());
                    ss >> temp;
                    ss >> m_tolerance;
                    break;
                }
            }
        } // while써監

		while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'C')
                {
                    string temp;
                    stringstream ss(line.GetString());
                    ss >> temp;
                    ss >> m_scale >> m_translation.m_x >> m_translation.m_y;
                    break;
                }
            }
        } // while써監

		while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'A')
                {
                    gb_SerializePolygon(ar, m_a);
                    break;
                }
            }
        } // while써監

		while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'B')
                {
                    gb_SerializePolygon(ar, m_b);
                    break;
                }
            }
        } // while써監
		CString string;
        CMFCRibbonBar* robbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar(); //삿혤Ribbon bar 얌깨
        if (robbon_bar==NULL)
            return;
        CMFCRibbonEdit* slider = (CMFCRibbonEdit*)robbon_bar->FindByID(ID_TOLERANCE); // 삿혤긍서움얌깨
        if (slider==NULL)
            return;
        string.Format("%g", m_tolerance);
        slider->SetEditText(string);
        CMFCRibbonComboBox* pbox = (CMFCRibbonComboBox*)robbon_bar->FindByID(ID_COMBO_AorB); // 삿혤긍서움얌깨
        if (pbox==NULL)
            return;
        pbox->AddItem("뜩긋近A");
        pbox->AddItem("뜩긋近B");
        pbox->SelectItem(0);
	}
}

// CCP_PolygonPlatformDoc 츱즈
void gb_SerializePolygon(CArchive& ar, CP_Polygon& p)
{
    CString line;
    unsigned int i, j, k;
    if (ar.IsStoring())
    {	// storing code
        line.Format("Pointsize %d\r\n", p.m_pointArray.size( ));
        ar.WriteString(line.GetString());
        for (i=0; i<p.m_pointArray.size( ); i++)
        {
            line.Format("%g %g\r\n", p.m_pointArray[i].m_x, p.m_pointArray[i].m_y);
            ar.WriteString(line.GetString());
        } // for써監
        line.Format("\r\nRegionsize %d\r\n", p.m_regionArray.size( ));
        ar.WriteString(line.GetString()); 
		for (i=0; i<p.m_regionArray.size( ); i++)
        {
            line.Format("Region %d\r\n", i);
            ar.WriteString(line.GetString());
            line.Format("Loopsize %d\r\n", p.m_regionArray[i].m_loopArray.size( ));
            ar.WriteString(line.GetString());
            for (j=0; j<p.m_regionArray[i].m_loopArray.size( ); j++)
            {
                line.Format("Loop %d\r\n", j);
                ar.WriteString(line.GetString());
                line.Format("PointIDsize %d\r\n", p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( ));
                ar.WriteString(line.GetString());
				for (k=0; k<p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( ); k++)
                {
                    line.Format("%d ", p.m_regionArray[i].m_loopArray[j].m_pointIDArray[k]);
                    ar.WriteString(line.GetString());
                } // for써監
                if (p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( )>0)
                {
                    line.Format("\r\n");
                    ar.WriteString(line.GetString());
                }
            } // for써監
        } // for써監
        line.Format("\r\n");
        ar.WriteString(line.GetString());
    }
	else
    {	// loading code
        p.m_pointArray.clear( );
        p.m_regionArray.clear( );
        while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'P')
                {
                    string temp;
                    stringstream ss(line.GetString());
                    ss >> temp;
                    ss >> i;
                    break;
                }
            }
        } // while써監
		if (i==0)
            return;
        p.m_pointArray.resize(i);
        for (i=0; i<p.m_pointArray.size( ); i++)
        {
            ar.ReadString(line);
            stringstream ss(line.GetString());
            ss >> p.m_pointArray[i].m_x;
            ss >> p.m_pointArray[i].m_y;
        } // for써監
		while(ar.ReadString(line))
        {
            if (!line.IsEmpty())
            {
                if (line[0] == 'R')
                {
                    string temp;
                    stringstream ss(line.GetString());
                    ss >> temp;
                    ss >> i;
                    break;
                }
            }
        } // while써監
		if (i==0)
            return;
        p.m_regionArray.resize(i);
        for (i=0; i<p.m_regionArray.size( ); i++)
        {
            p.m_regionArray[i].m_polygon = & p;
            p.m_regionArray[i].m_regionIDinPolygon = i;
            ar.ReadString(line);
            ar.ReadString(line);
            string temp;
            stringstream ss(line.GetString());
            ss >> temp;
            ss >> j;
            p.m_regionArray[i].m_loopArray.resize(j);
			for (j=0; j<p.m_regionArray[i].m_loopArray.size( ); j++)
            {
                p.m_regionArray[i].m_loopArray[j].m_polygon = & p;
                p.m_regionArray[i].m_loopArray[j].m_regionIDinPolygon = i;
                p.m_regionArray[i].m_loopArray[j].m_loopIDinRegion = j;
                ar.ReadString(line);
                ar.ReadString(line);
                string temp;
                stringstream ss(line.GetString());
                ss >> temp;
                ss >> k;
                p.m_regionArray[i].m_loopArray[j].m_pointIDArray.resize(k);
				if (k>0)
                {
                    ar.ReadString(line);
                    stringstream ss(line.GetString());
                    for (k=0; k<p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( ); k++)
                    {
                        ss >> p.m_regionArray[i].m_loopArray[j].m_pointIDArray[k];
                    } // for써監
                } // if써監
            } // for써監
        } // for써監
    } // 棍鍋if/else써監
} // 변鑒gb_SerializePolygon써監

#ifdef SHARED_HANDLERS

// 鍵쫠暠돨連넣
void CCP_PolygonPlatformDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 錦맣늪덜쯤鹿삥齡匡도鑒앴
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 鎧乞뇹잿넋埼돨連넣
void CCP_PolygonPlatformDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// 닒匡도鑒앴零鎧乞코휭。
	// 코휭꼬롸壇譚“;”롸몰

	// 절흔:  strSearchContent = _T("point;rectangle;circle;ole object;")；
	SetSearchContent(strSearchContent);
}

void CCP_PolygonPlatformDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CCP_PolygonPlatformDoc 閭뙤

#ifdef _DEBUG
void CCP_PolygonPlatformDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CCP_PolygonPlatformDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CCP_PolygonPlatformDoc 츱즈
