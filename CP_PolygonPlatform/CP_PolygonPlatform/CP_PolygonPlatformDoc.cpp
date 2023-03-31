#include "stdafx.h"
#include "CP_PolygonPlatform.h"
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

IMPLEMENT_DYNCREATE(CCP_PolygonPlatformDoc, CDocument)
BEGIN_MESSAGE_MAP(CCP_PolygonPlatformDoc, CDocument)
END_MESSAGE_MAP()

CCP_PolygonPlatformDoc::CCP_PolygonPlatformDoc(){mb_initData( );}

CCP_PolygonPlatformDoc::~CCP_PolygonPlatformDoc(){}

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
}
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
    pbox->AddItem("Polygon A");
    pbox->AddItem("Polygon B");
    pbox->SelectItem(0);

	return TRUE;
}

void CCP_PolygonPlatformDoc::Serialize(CArchive& ar)
{
	CString line;

	if (ar.IsStoring())
	{
		ar.WriteString("# Polygon Data\r\n");
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
        }

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
        }

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
        }

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
        }

		CString string;
        CMFCRibbonBar* robbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
        if (robbon_bar==NULL)
            return;
        CMFCRibbonEdit* slider = (CMFCRibbonEdit*)robbon_bar->FindByID(ID_TOLERANCE);
        if (slider==NULL)
            return;
        string.Format("%g", m_tolerance);
        slider->SetEditText(string);
        CMFCRibbonComboBox* pbox = (CMFCRibbonComboBox*)robbon_bar->FindByID(ID_COMBO_AorB);
        if (pbox==NULL)
            return;
        pbox->AddItem("Polygon A");
        pbox->AddItem("Polygon B");
        pbox->SelectItem(0);
	}
}

void gb_SerializePolygon(CArchive& ar, CP_Polygon& p)
{
    CString line;
    unsigned int i, j, k;
    if (ar.IsStoring())
    {	// storing code
        line.Format("Pointsize %d\r\n", p.m_pointArray.size());
        ar.WriteString(line.GetString());
        for (i = 0; i < p.m_pointArray.size(); i++)
        {
            line.Format("%g %g\r\n", p.m_pointArray[i].m_x, p.m_pointArray[i].m_y);
            ar.WriteString(line.GetString());
        }
        line.Format("\r\nRegionsize %d\r\n", p.m_regionArray.size());
        ar.WriteString(line.GetString());
        for (i = 0; i < p.m_regionArray.size(); i++)
        {
            line.Format("Region %d\r\n", i);
            ar.WriteString(line.GetString());
            line.Format("Loopsize %d\r\n", p.m_regionArray[i].m_loopArray.size());
            ar.WriteString(line.GetString());
            for (j = 0; j < p.m_regionArray[i].m_loopArray.size(); j++)
            {
                line.Format("Loop %d\r\n", j);
                ar.WriteString(line.GetString());
                line.Format("PointIDsize %d\r\n", p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size());
                ar.WriteString(line.GetString());
                for (k = 0; k < p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size(); k++)
                {
                    line.Format("%d ", p.m_regionArray[i].m_loopArray[j].m_pointIDArray[k]);
                    ar.WriteString(line.GetString());
                }
                if (p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size() > 0)
                {
                    line.Format("\r\n");
                    ar.WriteString(line.GetString());
                }
            }
        }
        line.Format("\r\n");
        ar.WriteString(line.GetString());
    }
    else
    {	// loading code
        p.m_pointArray.clear();
        p.m_regionArray.clear();
        while (ar.ReadString(line))
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
        }
        if (i == 0)
            return;
        p.m_pointArray.resize(i);
        for (i = 0; i < p.m_pointArray.size(); i++)
        {
            ar.ReadString(line);
            stringstream ss(line.GetString());
            ss >> p.m_pointArray[i].m_x;
            ss >> p.m_pointArray[i].m_y;
        }
        while (ar.ReadString(line))
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
        }
        if (i == 0)
            return;
        p.m_regionArray.resize(i);
        for (i = 0; i < p.m_regionArray.size(); i++)
        {
            p.m_regionArray[i].m_polygon = &p;
            p.m_regionArray[i].m_regionIDinPolygon = i;
            ar.ReadString(line);
            ar.ReadString(line);
            string temp;
            stringstream ss(line.GetString());
            ss >> temp;
            ss >> j;
            p.m_regionArray[i].m_loopArray.resize(j);
            for (j = 0; j < p.m_regionArray[i].m_loopArray.size(); j++)
            {
                p.m_regionArray[i].m_loopArray[j].m_polygon = &p;
                p.m_regionArray[i].m_loopArray[j].m_regionIDinPolygon = i;
                p.m_regionArray[i].m_loopArray[j].m_loopIDinRegion = j;
                ar.ReadString(line);
                ar.ReadString(line);
                string temp;
                stringstream ss(line.GetString());
                ss >> temp;
                ss >> k;
                p.m_regionArray[i].m_loopArray[j].m_pointIDArray.resize(k);
                if (k > 0)
                {
                    ar.ReadString(line);
                    stringstream ss(line.GetString());
                    for (k = 0; k < p.m_regionArray[i].m_loopArray[j].m_pointIDArray.size(); k++)
                    {
                        ss >> p.m_regionArray[i].m_loopArray[j].m_pointIDArray[k];
                    }
                }
            }
        }
    }
}

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