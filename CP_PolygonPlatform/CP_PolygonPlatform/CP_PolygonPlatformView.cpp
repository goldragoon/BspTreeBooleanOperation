#include "stdafx.h"
#include "CP_PolygonPlatform.h"
#include "CP_PolygonPlatformDoc.h"
#include "CP_PolygonPlatformView.h"
#include "MainFrm.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "CP_Polygon.h"

IMPLEMENT_DYNCREATE(CCP_PolygonPlatformView, CView)

BEGIN_MESSAGE_MAP(CCP_PolygonPlatformView, CView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_UPDATE_COMMAND_UI(ID_COMBO_AorB, &CCP_PolygonPlatformView::OnUpdateComboAorb)
	ON_COMMAND(ID_COMBO_AorB, &CCP_PolygonPlatformView::OnComboAorb)
	ON_COMMAND(ID_EDGE_NUMBER, &CCP_PolygonPlatformView::OnEdgeNumber)
	ON_COMMAND(ID_TOLERANCE, &CCP_PolygonPlatformView::OnTolerance)
	ON_COMMAND(ID_VIEW_STANDARD, &CCP_PolygonPlatformView::OnViewStandard)
	ON_COMMAND(ID_VIEW_FIT, &CCP_PolygonPlatformView::OnViewFit)
	ON_UPDATE_COMMAND_UI(ID_SELECT_POINT, &CCP_PolygonPlatformView::OnUpdateSelectPoint)
	ON_COMMAND(ID_SELECT_POINT, &CCP_PolygonPlatformView::OnSelectPoint)
	ON_UPDATE_COMMAND_UI(ID_SELECT_LOOP, &CCP_PolygonPlatformView::OnUpdateSelectLoop)
	ON_COMMAND(ID_SELECT_LOOP, &CCP_PolygonPlatformView::OnSelectLoop)
	ON_UPDATE_COMMAND_UI(ID_SELECT_REGION, &CCP_PolygonPlatformView::OnUpdateSelectRegion)
	ON_COMMAND(ID_SELECT_REGION, &CCP_PolygonPlatformView::OnSelectRegion)
	ON_UPDATE_COMMAND_UI(ID_SELECT_POLYGON, &CCP_PolygonPlatformView::OnUpdateSelectPolygon)
	ON_COMMAND(ID_SELECT_POLYGON, &CCP_PolygonPlatformView::OnSelectPolygon)
	ON_UPDATE_COMMAND_UI(ID_SELECT_TRIANGLE, &CCP_PolygonPlatformView::OnUpdateSelectTriangle)
	ON_COMMAND(ID_SELECT_TRIANGLE, &CCP_PolygonPlatformView::OnSelectTriangle)
	ON_UPDATE_COMMAND_UI(ID_SELECT_ONLY, &CCP_PolygonPlatformView::OnUpdateSelectOnly)
	ON_COMMAND(ID_SELECT_ONLY, &CCP_PolygonPlatformView::OnSelectOnly)
	ON_COMMAND(ID_NEW_RIGHT_OUTLOOP, &CCP_PolygonPlatformView::OnNewRightOutloop)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_COMMAND(ID_NEW_RIGHT_INLOOP, &CCP_PolygonPlatformView::OnNewRightInloop)
	ON_COMMAND(ID_ADD_OUTLOOP, &CCP_PolygonPlatformView::OnAddOutloop)
	ON_COMMAND(ID_ADD_INLOOP, &CCP_PolygonPlatformView::OnAddInloop)
	ON_COMMAND(ID_ADD_POINT, &CCP_PolygonPlatformView::OnAddPoint)
	ON_COMMAND(ID_DELETE, &CCP_PolygonPlatformView::OnDelete)
	ON_UPDATE_COMMAND_UI(ID_MOVE_SAME, &CCP_PolygonPlatformView::OnUpdateMoveSame)
	ON_COMMAND(ID_MOVE_SAME, &CCP_PolygonPlatformView::OnMoveSame)
	ON_UPDATE_COMMAND_UI(ID_VIEW_A, &CCP_PolygonPlatformView::OnUpdateViewA)
	ON_COMMAND(ID_VIEW_A, &CCP_PolygonPlatformView::OnViewA)
	ON_UPDATE_COMMAND_UI(ID_VIEW_B, &CCP_PolygonPlatformView::OnUpdateViewB)
	ON_COMMAND(ID_VIEW_B, &CCP_PolygonPlatformView::OnViewB)
	ON_UPDATE_COMMAND_UI(ID_VIEW_POINT_ID, &CCP_PolygonPlatformView::OnUpdateViewPointId)
	ON_COMMAND(ID_VIEW_POINT_ID, &CCP_PolygonPlatformView::OnViewPointId)
	ON_COMMAND(ID_CHECK, &CCP_PolygonPlatformView::OnCheck)
	
	ON_UPDATE_COMMAND_UI(ID_VIEW_RESULT, &CCP_PolygonPlatformView::OnUpdateViewResult)
	ON_COMMAND(ID_VIEW_RESULT, &CCP_PolygonPlatformView::OnViewResult)

    // Boolean Operation Button 
    ON_COMMAND(ID_POLYGON_UNION, &CCP_PolygonPlatformView::OnPolygonUnion)
	ON_COMMAND(ID_POLYGON_INTERSECTION, &CCP_PolygonPlatformView::OnPolygonIntersection)
	ON_COMMAND(ID_POLYGON_A_B, &CCP_PolygonPlatformView::OnPolygonAB)
	ON_COMMAND(ID_POLYGON_B_A, &CCP_PolygonPlatformView::OnPolygonBA)
	ON_WM_ERASEBKGND()
END_MESSAGE_MAP()

CCP_PolygonPlatformView::CCP_PolygonPlatformView()
{
	m_buildTime = 0;
	m_mergeTime = 0;
	m_generateEdgeTime = 0;
}
CCP_PolygonPlatformView::~CCP_PolygonPlatformView(){}

BOOL CCP_PolygonPlatformView::PreCreateWindow(CREATESTRUCT& cs)
{
	return CView::PreCreateWindow(cs);
}

void CCP_PolygonPlatformView::OnDraw(CDC* pDC)
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	CRect r;
    GetClientRect(& r);

    if (!pDoc->m_flagShowSelect)
    {
        if (pDoc->m_flagShowA)
        {
            gb_drawPolygonLoop(pDC, pDoc->m_a,
                pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                255, 0, 0,
                0, 255, 0,
                3);
            gb_drawPolygonPoint(pDC, pDoc->m_a,
                pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                0, 0, 0,
                3);
            if (pDoc->m_flagShowPointID)
                gb_drawPolygonPointID(pDC, pDoc->m_a,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    0, 0, 0);
        }

        if (pDoc->m_flagShowB)
        {
            gb_drawPolygonLoop(pDC, pDoc->m_b,
                pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                255, 0, 255,
                0, 0, 255,
                3);
            gb_drawPolygonPoint(pDC, pDoc->m_b,
                pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                0, 0, 0,
                3);
            if (pDoc->m_flagShowPointID)
                gb_drawPolygonPointID(pDC, pDoc->m_b,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    0, 0, 255);
        }
    }

    if (pDoc->m_flagSelect) // 显示选择集
    {
        switch(pDoc->m_flagSelectType)
        {
        case 1: // 点。
            if (pDoc->m_flagSelectPolygon==0)
                gb_drawPointGlobal(pDC, pDoc->m_a.m_pointArray[pDoc->m_flagSelectID],
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    50, 50, 50,
                    6); // A
            else gb_drawPointGlobal(pDC, pDoc->m_b.m_pointArray[pDoc->m_flagSelectID],
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    50, 50, 50,
                    6); // B
            break;
		case 2: // 环。
            if (pDoc->m_flagSelectPolygon==0)
            {
                if (pDoc->m_flagSelectID==0)
                    gb_drawLoop(pDC, pDoc->m_a.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[pDoc->m_flagSelectID],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        255, 100, 100,
                        5); // 外环
                else
                    gb_drawLoop(pDC, pDoc->m_a.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[pDoc->m_flagSelectID],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        100, 255, 50,
                        5); // 内环
            } // A
			else
            {
                if (pDoc->m_flagSelectID==0)
                    gb_drawLoop(pDC, pDoc->m_b.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[pDoc->m_flagSelectID],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        200, 100, 200,
                        5); // 外环
                else
                    gb_drawLoop(pDC, pDoc->m_b.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[pDoc->m_flagSelectID],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        100, 50, 255,
                        5); // 内环
            } // B
            break;
		case 3: // 区域。
            int i, n;
            if (pDoc->m_flagSelectPolygon==0)
            {
                n = pDoc->m_a.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray.size( );
                for (i=0; i<n; i++)
                if (i==0)
                    gb_drawLoop(pDC, pDoc->m_a.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[i],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        255, 100, 100,
                        5); // 外环
                else
                    gb_drawLoop(pDC, pDoc->m_a.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[i],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        100, 255, 50,
                        5); // 内环
            } // A
           else
            {
                n = pDoc->m_b.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray.size( );
                for (i=0; i<n; i++)
                if (i==0)
                    gb_drawLoop(pDC, pDoc->m_b.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[i],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        200, 100, 200,
                        5); // 外环
                else
                    gb_drawLoop(pDC, pDoc->m_b.m_regionArray[pDoc->m_flagSelectRegion].m_loopArray[i],
                        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                        100, 50, 255,
                        5); // 内环
            } // B
            break;
        case 4: // 多边形。
            if (pDoc->m_flagSelectPolygon==0)
            {
                gb_drawPolygonLoop(pDC, pDoc->m_a,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    255, 100, 100,
                    100, 255, 50,
                    5);
                gb_drawPolygonPoint(pDC, pDoc->m_a,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    50, 50, 50,
                    6);
            } // A
           else
            {
                gb_drawPolygonLoop(pDC, pDoc->m_b,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    200, 100, 200,
                    100, 50, 255,
                    5);
                gb_drawPolygonPoint(pDC, pDoc->m_b,
                    pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
                    50, 50, 50,
                    6);
            } // B
            break;
        case 5: // 剖分三角形。
            break;
        } // switch结束
    }

    if (pDoc->m_flagAdd == 1)
    {
        gb_drawPointArrayLine(pDC, pDoc->m_flagAddPointArray,
            pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
            200, 0, 255,
            5);
        gb_drawPointArrayPoint(pDC, pDoc->m_flagAddPointArray,
            pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
            0, 0, 0,
            6);
    }

    if (pDoc->m_flagAdd == 2)
    {
        gb_drawPointArrayLine(pDC, pDoc->m_flagAddPointArray,
            pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
            0, 200, 255,
            5);
        gb_drawPointArrayPoint(pDC, pDoc->m_flagAddPointArray,
            pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
            0, 0, 0,
            6);
    }

	if(pDoc->m_showBsptree){
		//if(!pDoc->m_bspTree) 
		gb_drawBspTree(pDC, pDoc->m_bspTree,
            pDoc->m_scale, pDoc->m_translation, r.right, r.bottom,
        0, 200, 255, 4);
	}
	
    //display process time
    CRect rect;
    GetClientRect(&rect);
    CString textout;
    textout.Format("Build BspTree time: %.2lf ms.", m_buildTime);
    pDC->TextOutA(rect.right - 250, rect.top + 30, textout.GetString());
    textout.Format("merge time: %.2lf ms.", m_mergeTime);
    pDC->TextOutA(rect.right - 250, rect.top + 50, textout.GetString());
    textout.Format("Generate edge time: %.2lf ms.", m_generateEdgeTime);
    pDC->TextOutA(rect.right - 250, rect.top + 70, textout.GetString());
}

void CCP_PolygonPlatformView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
}

#ifdef _DEBUG
void CCP_PolygonPlatformView::AssertValid() const
{
	CView::AssertValid();
}

void CCP_PolygonPlatformView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CCP_PolygonPlatformDoc* CCP_PolygonPlatformView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CCP_PolygonPlatformDoc)));
	return (CCP_PolygonPlatformDoc*)m_pDocument;
}
#endif //_DEBUG

void  CCP_PolygonPlatformView::mb_statusSetText(char* s1, char* s2)
{
    CRect r;
    // View中获取MainFrame指针
    CMainFrame *pMainFrame=(CMainFrame *)AfxGetApp()->m_pMainWnd;
    //获取状态栏指针
    CMFCRibbonStatusBar *statusBar=
        (CMFCRibbonStatusBar *)pMainFrame->GetDescendantWindow(AFX_IDW_STATUS_BAR);
    //获取RibbonStatusBar上的元素
    CMFCRibbonLabel *locLabel;
    if (s1!=NULL)
    {
        locLabel=(CMFCRibbonLabel *)statusBar->GetElement(0);
        if (locLabel!=NULL)
        {
            r = locLabel->GetRect( );
            if (r.right-r.left<300)
                r.right+=300;
            locLabel->SetRect(&r);
            locLabel->SetText(s1);
        } // if结束
    } // 外if结束
   if (s2!=NULL)
    {
        locLabel=(CMFCRibbonLabel *)statusBar->GetExElement(0);
        if (locLabel!=NULL)
        {
            r = locLabel->GetRect( );
            if (r.right-r.left<300)
                r.left-=300;
            locLabel->SetRect(&r);
            locLabel->SetText(s2);
        } // if结束
    } // 外if结束
    statusBar->Invalidate( );
} // 类CCP_PolygonPlatformView的成员函数mb_statusSetText结束

void gb_drawLoop(CDC* pDC, CP_Loop& p, 
                 double scale, CP_Point2 translation, int screenX, int screenY, 
                 int r, int g, int b, int size)
{
    int n = p.m_pointIDArray.size( );
    if (n<=0)
        return;
    CPen pen(0, size, RGB(r, g, b));
    CPen * penOld = (CPen *) pDC->SelectObject(&pen);
    int i, k;
    i = p.m_pointIDArray[n-1];
    CP_Point2 pg = p.m_polygon->m_pointArray[i];
    CP_Point2 ps;
    gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
    pDC->MoveTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));
    for (k=0; k<n; k++)
    {
        i = p.m_pointIDArray[k];
        pg = p.m_polygon->m_pointArray[i];
        gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
        pDC->LineTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));
    }
    pDC->SelectObject(penOld);
}

void gb_drawPointArrayLine(CDC* pDC, VT_PointArray& pa,
        double scale, CP_Point2 translation, int screenX, int screenY,
        int r, int g, int b, int size)
{
    int n = pa.size( );
    if (n<=0)
        return;
    CPen pen(0, size, RGB(r, g, b));
    CPen * penOld = (CPen *) pDC->SelectObject(&pen);
    int i;
    CP_Point2 pg = pa[0];
    CP_Point2 ps;
    gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
    pDC->MoveTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));
    for (i=0; i<n; i++)
    {
        pg = pa[i];
        gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
        pDC->LineTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));
    }
    pDC->SelectObject(penOld);
} 

void gb_drawBspTree(CDC* pDC, CP_BSPNode* tree,
        double scale, CP_Point2 translation, int screenX, int screenY,
        int r, int g, int b, int size)
{
    if (tree == NULL)
        return;

	if(tree->isCell()){
		return;
	}
	else{
		gb_drawBspNode(pDC, tree, scale, translation, screenX, screenY, r, g, b, size);
        gb_drawBspTree(pDC, tree->leftChild, scale, translation, screenX, screenY, r, g, b, size);
        gb_drawBspTree(pDC, tree->rightChild, scale, translation, screenX, screenY, r, g, b, size);
	}
}

void gb_drawBspNode(CDC* pDC, CP_BSPNode* tree, double scale, CP_Point2 translation, int screenX, int screenY, int r, int g, int b, int size)
{
	CP_BSPNode* node = tree;
	CP_Point2 ps;
	CPen pen(0, size, RGB(r, g, b));
    CPen * penOld = (CPen *) pDC->SelectObject(&pen);
	CP_Point2 pg;

	for(unsigned int i = 0; i < node->polygon.size(); i++){
		pg= node->polygon[i].begin;
		gb_drawPointGlobal(pDC, pg, scale, translation, screenX, screenY, 255, 255, 255, size - 1);
		gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
		pDC->MoveTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));

		//pg = node->partition->end;
		pg = node->polygon[i].end;
		gb_drawPointGlobal(pDC, pg, scale, translation, screenX, screenY, 255, 255, 255, size - 1);
		gb_pointConvertFromGlobalToScreen(ps, pg, scale, translation, screenX, screenY);
		pDC->LineTo((int)(ps.m_x+0.5), (int)(ps.m_y+0.5));
	}
	pDC->SelectObject(penOld);
}

void gb_drawPointArrayPoint(CDC* pDC, VT_PointArray& pa,
        double scale, CP_Point2 translation, int screenX, int screenY,
        int r, int g, int b, int size)
{
    int n = pa.size( );
    int i;
    for (i=0; i<n; i++)
    {
        gb_drawPointGlobal(pDC, pa[i], scale, translation, screenX, screenY, r, g, b, size);
    } // for结束
} // 函数gb_drawPointArrayPoint结束

void gb_drawPointGlobal(CDC* pDC, CP_Point2 pointGlobal, double scale, CP_Point2 translation, int screenX, int screenY, int r, int g, int b, int size)
{
    CP_Point2 ps;
    gb_pointConvertFromGlobalToScreen(ps, pointGlobal, scale, translation,screenX, screenY);
    gb_drawPointScreen(pDC, (int)(ps.m_x+0.5), (int)(ps.m_y+0.5), r, g, b, size);
} // 函数gb_drawPointScreen结束

void gb_drawPointScreen(CDC* pDC, int x, int y, int r, int g, int b, int size)
{
    CBrush brush(RGB(r, g, b));
    CBrush* brushOld = (CBrush*)pDC->SelectObject(&brush);;
    // 绘制格子
    CRect rect(x - size, y - size, x + size, y + size);
    pDC->Ellipse(&rect);
    pDC->SelectObject(brushOld);
} // 函数gb_drawPointScreen结束

void gb_drawPolygonLoop(CDC* pDC, CP_Polygon& p,
                        double scale, CP_Point2 translation, int screenX, int screenY,
                        int outR, int outG, int outB, 
                        int inR,  int inG,  int inB, 
                        int size)
{
    unsigned int i, k;
    for (i=0; i<p.m_regionArray.size( ); i++)
    {
        for (k=0; k<p.m_regionArray[i].m_loopArray.size( ); k++)
        {
            if (k==0)
                gb_drawLoop(pDC, p.m_regionArray[i].m_loopArray[k], 
                            scale, translation,screenX, screenY, 
                            outR, outG, outB,size);
            else
                gb_drawLoop(pDC, p.m_regionArray[i].m_loopArray[k], 
                            scale, translation,screenX, screenY, 
                            inR, inG, inB,size);
        } // 内部for结束
    } // 外部for结束
} // 函数gb_drawPolygonLoop结束

void gb_drawPolygonPoint(CDC* pDC, CP_Polygon& p, double scale, CP_Point2 translation, int screenX, int screenY, int r, int g, int b, int size)
{
    int n = p.m_pointArray.size( );
    int i;
    for (i=0; i<n; i++)
    {
        gb_drawPointGlobal(pDC, p.m_pointArray[i], scale, translation, screenX, screenY, r, g, b, size);
    } // for结束
} // 函数gb_drawPolygonPoint结束

void gb_drawPolygonPointID(CDC* pDC, CP_Polygon& p,
            double scale, CP_Point2 translation, int screenX, int screenY,
            int r, int g, int b)
{
    COLORREF cOld = pDC->SetTextColor(RGB(r, g, b));
    int nr = p.m_regionArray.size( );
    int nL, nLv, ir, iL, iLv, v;
    CP_Point2 ps;
    char buffer[100];
    for (ir=0; ir<nr; ir++)
    {
        nL = p.m_regionArray[ir].m_loopArray.size( );
       for (iL=0; iL<nL; iL++)
        {
            nLv =  p.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.size( );
            for (iLv=0; iLv<nLv; iLv++)
            {
                v = p.m_regionArray[ir].m_loopArray[iL].m_pointIDArray[iLv];
                gb_pointConvertFromGlobalToScreen(ps, p.m_pointArray[v], 
                    scale, translation,screenX, screenY);
                sprintf_s(buffer, 100, "[%1d]R%1dL%1dV%1d", v, ir, iL, iLv);
                pDC->TextOutA((int)(ps.m_x+0.5), (int)(ps.m_y+0.5), buffer);
            } // for(iLv)结束
        } // for(iL)结束
    } // for(ir)结束
    pDC->SetTextColor(cOld);
} // 函数gb_drawPolygonPointID结束

void CCP_PolygonPlatformView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
//	OnContextMenu(this, point);
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagAdd == 3)
    {
        pDoc->m_flagAdd = 0;
        mb_statusSetText("点添加操作结束。", "请继续其他操作");
        return;
    } // if结束
    if ((pDoc->m_flagAdd == 1) || (pDoc->m_flagAdd == 2)) // 环
    {
        int n = pDoc->m_flagAddPointArray.size( );
        if (n<=0)
        {
            pDoc->m_flagAdd = 0;
            mb_statusSetText("环添加操作结束。", "请继续其他操作");
            return;
        } // if结束
       if (n<3)
        {
            if (MessageBox("构成环的点不能少于3个，是否继续输入? 选是则继续，否则放弃前面输入的点。"
                      , "操作确认"
                      , MB_ICONQUESTION|MB_YESNO)==IDYES)
                return;
            if (pDoc->m_flagAdd == 1)
                 mb_statusSetText("外环点添加操作结束。", "请继续其他操作");
            else mb_statusSetText("内环点添加操作结束。", "请继续其他操作");
            pDoc->m_flagAdd = 0;
            pDoc->m_flagAddPointArray.clear( );
            return;
        } // if结束
       int i, k;
        CP_Polygon* pn;
        if (pDoc->m_flagAddIDPolygon==0)
             pn= &(pDoc->m_a);
        else pn= &(pDoc->m_b);
        int nv = pn->m_pointArray.size( );
        pn->m_pointArray.resize(nv+n);
        for (i=0, k=nv; i<n; i++, k++)
        {
            pn->m_pointArray[k].m_x = pDoc->m_flagAddPointArray[i].m_x;
            pn->m_pointArray[k].m_y = pDoc->m_flagAddPointArray[i].m_y;
        } // for结束
       if (pDoc->m_flagAdd == 1)
        {
            pDoc->m_flagAddIDLoop=0;
            pDoc->m_flagAddIDRegion=pn->m_regionArray.size( );
            pn->m_regionArray.resize(pDoc->m_flagAddIDRegion+1);
            pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.resize(1);
            pn->m_regionArray[pDoc->m_flagAddIDRegion].m_polygon=pn;
            pn->m_regionArray[pDoc->m_flagAddIDRegion].m_regionIDinPolygon = pDoc->m_flagAddIDRegion;
        }
        else
        {
            pDoc->m_flagAddIDLoop= pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( );
            pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.resize(pDoc->m_flagAddIDLoop+1);
        } // if/else结束
        pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_polygon = pn;

        pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_regionIDinPolygon = pDoc->m_flagAddIDRegion;

        pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_loopIDinRegion = pDoc->m_flagAddIDLoop;

        pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray.resize(n);

        for (i=0, k= nv; i<n; i++, k++)
        {
            pn->m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray[i] = k;
        } // for结束
       if (pDoc->m_flagAdd == 1)
             mb_statusSetText("外环点添加操作结束。", "请继续其他操作");
        else mb_statusSetText("内环点添加操作结束。", "请继续其他操作");
        pDoc->m_flagAdd = 0;
        pDoc->m_flagAddPointArray.clear( );
        Invalidate( );
        return;
    } // 外if结束
}

void CCP_PolygonPlatformView::OnUpdateComboAorb(CCmdUI *pCmdUI)
{
    CMFCRibbonBar* ribbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
    if (ribbon_bar==NULL)
        return;
    CMFCRibbonComboBox* pbox = (CMFCRibbonComboBox*)ribbon_bar->FindByID(ID_COMBO_AorB); // 获取编辑框句柄
    if (pbox==NULL)
        return;
    pbox->AddItem("Polygon A");
    pbox->AddItem("Polygon B");

}

void CCP_PolygonPlatformView::OnComboAorb()
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    CMFCRibbonBar* ribbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
    if (ribbon_bar==NULL)
        return;
    CMFCRibbonComboBox* pbox = (CMFCRibbonComboBox*)ribbon_bar->FindByID(ID_COMBO_AorB);
    if (pbox==NULL)
        return;
    pbox->AddItem("Polygon A");
    pbox->AddItem("Polygon B");
    int i = pbox->GetCurSel( );
    pbox->SelectItem(i);
    if (i==0)
        pDoc->m_flagBuildA = true;
    else pDoc->m_flagBuildA = false;
    Invalidate( );

}


void CCP_PolygonPlatformView::OnEdgeNumber()
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    CString string;
    CMFCRibbonBar* ribbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
    if (ribbon_bar==NULL)
        return;
    CMFCRibbonEdit* slider = (CMFCRibbonEdit*)ribbon_bar->FindByID(ID_EDGE_NUMBER);
    if (slider==NULL)
        return;
    string= slider->GetEditText(); // 获取数字
    int i = atoi(string);
    if (i<3)
        i = 3;
    if (i>10000)
        i=10000;
    pDoc->m_edgeNumber = i;
    string.Format("%d", i);
    slider->SetEditText(string);
    Invalidate( );
}

void CCP_PolygonPlatformView::OnTolerance()
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    CString string;
    CMFCRibbonBar* ribbon_bar = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar(); //获取Ribbon bar 句柄
    if (ribbon_bar==NULL)
        return;
    CMFCRibbonEdit* slider = (CMFCRibbonEdit*)ribbon_bar->FindByID(ID_TOLERANCE); // 获取编辑框句柄
    if (slider==NULL)
        return;
    string= slider->GetEditText(); // 获取数字
    double d = atof(string);
    if (d<=0.0)
        d = 1e-6;
    pDoc->m_tolerance = d;
    string.Format("%g", pDoc->m_tolerance);
    slider->SetEditText(string);
    Invalidate( );

}

void CCP_PolygonPlatformView::OnViewStandard()
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_scale = 1.0; // 缩放比例
    pDoc->m_translation.m_x = 0.0; // 坐标平移量
    pDoc->m_translation.m_y = 0.0; // 坐标平移量
    Invalidate( );
    mb_statusSetText("标准化坐标系。", "不平移，也不缩放。");
}

void CCP_PolygonPlatformView::OnViewFit()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    double dxMin, dyMin, dxMax, dyMax, ra, s1, s2;
    int na = pDoc->m_a.m_pointArray.size( );
    int nb = pDoc->m_b.m_pointArray.size( );
    if ((na==0) && (nb==0))
        return;
	if (na!=0)
    {
        dxMin = pDoc->m_a.m_pointArray[0].m_x;
        dxMax = dxMin;
        dyMin = pDoc->m_a.m_pointArray[0].m_y;
        dyMax = dyMin;
    }
    else if (nb!=0)
    {
        dxMin = pDoc->m_b.m_pointArray[0].m_x;
        dxMax = dxMin;
        dyMin = pDoc->m_b.m_pointArray[0].m_y;
        dyMax = dyMin;
    }
	int i;
    for (i=0; i<na; i++)
    {
        if (dxMin>pDoc->m_a.m_pointArray[i].m_x)
            dxMin=pDoc->m_a.m_pointArray[i].m_x;
        if (dxMax<pDoc->m_a.m_pointArray[i].m_x)
            dxMax=pDoc->m_a.m_pointArray[i].m_x;
        if (dyMin>pDoc->m_a.m_pointArray[i].m_y)
            dyMin=pDoc->m_a.m_pointArray[i].m_y;
        if (dyMax<pDoc->m_a.m_pointArray[i].m_y)
            dyMax=pDoc->m_a.m_pointArray[i].m_y;
    } // for结束
    for (i=0; i<nb; i++)
    {
        if (dxMin>pDoc->m_b.m_pointArray[i].m_x)
            dxMin=pDoc->m_b.m_pointArray[i].m_x;
        if (dxMax<pDoc->m_b.m_pointArray[i].m_x)
            dxMax=pDoc->m_b.m_pointArray[i].m_x;
        if (dyMin>pDoc->m_b.m_pointArray[i].m_y)
            dyMin=pDoc->m_b.m_pointArray[i].m_y;
        if (dyMax<pDoc->m_b.m_pointArray[i].m_y)
            dyMax=pDoc->m_b.m_pointArray[i].m_y;
    } // for结束
    CRect r;
    GetClientRect(& r);
    r.bottom -=40;
    r.right -=40;
    pDoc->m_translation.m_x=(dxMin+dxMax)/2;
    pDoc->m_translation.m_y=(dyMin+dyMax)/2;
    ra=(double)(dxMax-dxMin);
    if (ra<10e-8)
        ra=1;
    s1=(double)(r.right-r.left)/ra;
    ra=(double)(dyMax-dyMin);
    if (ra<10e-8)
        ra=1;
    s2=(double)(r.bottom-r.top)/ra;
    pDoc->m_scale=(s1<s2 ? s1 : s2);
    Invalidate( );
    mb_statusSetText("自适应显示!", "尽量充满屏幕!");

}


void CCP_PolygonPlatformView::OnUpdateSelectPoint(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==1)
        pCmdUI->SetCheck(true);
    else pCmdUI->SetCheck(false);

}


void CCP_PolygonPlatformView::OnSelectPoint()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==1)
        pDoc->m_flagSelectType = 0;
    else pDoc->m_flagSelectType = 1;
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateSelectLoop(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==2)
        pCmdUI->SetCheck(true);
    else pCmdUI->SetCheck(false);

}


void CCP_PolygonPlatformView::OnSelectLoop()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==2)
        pDoc->m_flagSelectType = 0;
    else pDoc->m_flagSelectType = 2;
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateSelectRegion(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==3)
        pCmdUI->SetCheck(true);
    else pCmdUI->SetCheck(false);

}


void CCP_PolygonPlatformView::OnSelectRegion()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==3)
        pDoc->m_flagSelectType = 0;
    else pDoc->m_flagSelectType = 3;
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateSelectPolygon(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==4)
        pCmdUI->SetCheck(true);
    else pCmdUI->SetCheck(false);

}


void CCP_PolygonPlatformView::OnSelectPolygon()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==4)
        pDoc->m_flagSelectType = 0;
    else pDoc->m_flagSelectType = 4;
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateSelectTriangle(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==5)
        pCmdUI->SetCheck(true);
    else pCmdUI->SetCheck(false);

}


void CCP_PolygonPlatformView::OnSelectTriangle()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagSelectType==5)
        pDoc->m_flagSelectType = 0;
    else pDoc->m_flagSelectType = 5;
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateSelectOnly(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pCmdUI->SetCheck(pDoc->m_flagShowSelect);

}


void CCP_PolygonPlatformView::OnSelectOnly()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_flagShowSelect ^= true;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnNewRightOutloop()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    CRect r;
    GetClientRect(& r);
    double dr = (r.right<r.bottom ? r.right : r.bottom);
    dr /=3;
    if (pDoc->m_flagBuildA)
         gb_polygonNewOutLoopRegular(pDoc->m_a, pDoc->m_edgeNumber, dr, 0.0, 0.0);
    else gb_polygonNewOutLoopRegular(pDoc->m_b, pDoc->m_edgeNumber, dr, 0.0, 0.0);
    Invalidate( );
    char s[100];
    sprintf_s(s, 100, "新外环是正%1d边形", pDoc->m_edgeNumber);
    if (pDoc->m_flagBuildA)
         mb_statusSetText("在多边形A中创建了新外环。", s);
    else mb_statusSetText("在多边形B中创建了新外环。", s);

}


void CCP_PolygonPlatformView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    CP_Point2 ps, pg;
    ps.m_x = point.x;
    ps.m_y = point.y;
    CRect r;
    GetClientRect(& r);
    gb_pointConvertFromScreenToGlobal(pg, ps, 
        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom);
    pDoc->m_basePoint = pg;
    bool flagSuceess = false;
    CP_Polygon* pn0;
	if (pDoc->m_flagAddIDPolygon==0)
         pn0 = &(pDoc->m_a);
    else pn0 = &(pDoc->m_b);
    if (pDoc->m_flagAdd == 3)
    {
        gb_insertPointInPolygon(*pn0, pDoc->m_flagAddIDRegion, pDoc->m_flagAddIDLoop, pDoc->m_flagAddIDPointInLoop, pg);
        mb_statusSetText("点添加操作成功。", "用鼠标右键结束点添加操作");
        Invalidate( );
        CView::OnLButtonDown(nFlags, point);
        return;
    } // 外if结束
	if ((pDoc->m_flagAdd == 1) || (pDoc->m_flagAdd == 2)) // 环
    {
        pDoc->m_flagAddPointArray.push_back(pg);
        if (pDoc->m_flagAdd == 1)
             mb_statusSetText("外环点添加操作成功。", "用鼠标右键结束外环添加操作");
        else mb_statusSetText("内环点添加操作成功。", "用鼠标右键结束内环添加操作");
        Invalidate( );
        CView::OnLButtonDown(nFlags, point);
        return;
    } // 外if结束

    if ((!(pDoc->m_flagShowA))&&(!(pDoc->m_flagShowB)))
    {
        CView::OnLButtonDown(nFlags, point);
        return;
    } // if结束
	double da, db;
    int ida, idb, ira, irb;
    CP_Polygon* pn1;
    VT_IntArray* pSet0;
    VT_IntArray* pSet1;
    CP_Point2 p0, p1;
    switch(pDoc->m_flagSelectType)
    {
    case 1: // 点。
        if (pDoc->m_flagShowA)
            gb_distanceMinPointPolygon(da, ida, pg, pDoc->m_a);
        else ida = -1;
        if (pDoc->m_flagShowB)
            gb_distanceMinPointPolygon(db, idb, pg, pDoc->m_b);
        else idb = -1;
		if (ida>=0)
        {
            if (idb>=0)
            {
                if (da<=db)
                {
                    pDoc->m_flagSelect = true;
                    pDoc->m_flagSelectPolygon = 0;
                    pDoc->m_flagSelectID = ida;
                }
                else
                {
                    pDoc->m_flagSelect = true;
                    pDoc->m_flagSelectPolygon = 1;
                    pDoc->m_flagSelectID = idb;
                }
            }
            else
            {
                pDoc->m_flagSelect = true;
                pDoc->m_flagSelectPolygon = 0;
                pDoc->m_flagSelectID = ida;
            }
        }
        else
        {
            if (idb>=0)
            {
                pDoc->m_flagSelect = true;
                pDoc->m_flagSelectPolygon = 1;
                pDoc->m_flagSelectID = idb;
            }
            else pDoc->m_flagSelect = false;
        }
        if (pDoc->m_flagSelect)
        {
            if (pDoc->m_flagMoveSame)
            {
                if (pDoc->m_flagSelectPolygon==0)
                {
                    pn0 = &(pDoc->m_a);
                    pn1 = &(pDoc->m_b);
                    pSet0 = &(pDoc->m_flagSelectIDSetInA);
                    pSet1 = &(pDoc->m_flagSelectIDSetInB);
                }
                else
                {
                    pn0 = &(pDoc->m_b);
                    pn1 = &(pDoc->m_a);
                    pSet0 = &(pDoc->m_flagSelectIDSetInB);
                    pSet1 = &(pDoc->m_flagSelectIDSetInA);
                } // if/else结束
                gb_intArrayInitPoint(*pSet0, *pn0, pDoc->m_flagSelectID, pDoc->m_tolerance);
                gb_intArrayInitPolygonSamePoint(*pSet1, *pn1, *pSet0, *pn0, pDoc->m_tolerance);
            } // if结束
        } // if结束
        break;
    case 2: // 环。
    case 3: // 区域。
    case 4: // 多边形。
        if (pDoc->m_flagShowA)
            gb_distanceMinPointLoop(da, ira, ida, pg, pDoc->m_a);
        else ida = -1;
        if (pDoc->m_flagShowB)
            gb_distanceMinPointLoop(db, irb, idb, pg, pDoc->m_b);
        else idb = -1;
		if (ida>=0)
        {
            if (idb>=0)
            {
                if (da<=db)
                {
                    pDoc->m_flagSelect = true;
                    pDoc->m_flagSelectPolygon = 0;
                    pDoc->m_flagSelectRegion = ira;
                    pDoc->m_flagSelectID = ida;
                }
                else
                {
                    pDoc->m_flagSelect = true;
                    pDoc->m_flagSelectPolygon = 1;
                    pDoc->m_flagSelectRegion = irb;
                    pDoc->m_flagSelectID = idb;
                }
            }
            else
            {
                pDoc->m_flagSelect = true;
                pDoc->m_flagSelectPolygon = 0;
                pDoc->m_flagSelectRegion = ira;
                pDoc->m_flagSelectID = ida;
            }
        }
        else
        {
            if (idb>=0)
            {
                pDoc->m_flagSelect = true;
                pDoc->m_flagSelectPolygon = 1;
                pDoc->m_flagSelectRegion = irb;
                pDoc->m_flagSelectID = idb;
            }
            else pDoc->m_flagSelect = false;
        }
       if (pDoc->m_flagSelect)
        {
            if (pDoc->m_flagMoveSame)
            {
                if (pDoc->m_flagSelectPolygon==0)
                {
                    pn0 = &(pDoc->m_a);
                    pn1 = &(pDoc->m_b);
                    pSet0 = &(pDoc->m_flagSelectIDSetInA);
                    pSet1 = &(pDoc->m_flagSelectIDSetInB);
                }
                else
                {
                    pn0 = &(pDoc->m_b);
                    pn1 = &(pDoc->m_a);
                    pSet0 = &(pDoc->m_flagSelectIDSetInB);
                    pSet1 = &(pDoc->m_flagSelectIDSetInA);
                } // if/else结束
                if (pDoc->m_flagSelectType==4)
                    gb_intArrayInitPolygon(*pSet0, *pn0);
                else if (pDoc->m_flagSelectType==3)
                    gb_intArrayInitRegion(*pSet0, *pn0, pDoc->m_flagSelectRegion, pDoc->m_tolerance);
                else if (pDoc->m_flagSelectType==2)
                    gb_intArrayInitLoop(*pSet0, *pn0, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID, pDoc->m_tolerance);
                gb_intArrayInitPolygonSamePoint(*pSet1, *pn1, *pSet0, *pn0, pDoc->m_tolerance);
            } // if结束
        } // if结束
        break;
    case 5: // 剖分三角形。
        break;
    } // switch结束

	CView::OnLButtonDown(nFlags, point);
    Invalidate( );
    if (pDoc->m_flagSelect)
        pDoc->m_flagMouseDown = true;

}


void CCP_PolygonPlatformView::OnLButtonUp(UINT nFlags, CPoint point)
{
	CView::OnLButtonUp(nFlags, point);

	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (!pDoc->m_flagSelect)
        return;
    if (!pDoc->m_flagMouseDown)
        return;
    CP_Point2 ps, pg;
    ps.m_x = point.x;
    ps.m_y = point.y;
    CRect r;
    GetClientRect(& r);
    gb_pointConvertFromScreenToGlobal(pg, ps, 
        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom);
    double vx = pg.m_x - pDoc->m_basePoint.m_x;
    double vy = pg.m_y - pDoc->m_basePoint.m_y;
    if (pDoc->m_flagMoveSame)
    {
        gb_movePointIntArray(pDoc->m_a, pDoc->m_flagSelectIDSetInA, vx, vy);
        gb_movePointIntArray(pDoc->m_b, pDoc->m_flagSelectIDSetInB, vx, vy);
        pDoc->m_basePoint = pg;
        Invalidate( );
        pDoc->m_flagMouseDown = false;
        return;
    } // if结束
   switch(pDoc->m_flagSelectType)
    {
    case 1: // 点。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_movePoint(pDoc->m_a, pDoc->m_flagSelectID, vx, vy);
        else gb_movePoint(pDoc->m_b, pDoc->m_flagSelectID, vx, vy);
        break;
    case 2: // 环。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_moveLoop(pDoc->m_a, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID, vx, vy);
        else gb_moveLoop(pDoc->m_b, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID, vx, vy);
        break;
    case 3: // 区域。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_moveRegion(pDoc->m_a, pDoc->m_flagSelectRegion, vx, vy);
        else gb_moveRegion(pDoc->m_b, pDoc->m_flagSelectRegion, vx, vy);
        break;
    case 4: // 多边形。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_movePolygon(pDoc->m_a, vx, vy);
        else gb_movePolygon(pDoc->m_b, vx, vy);
        break;
    case 5: // 剖分三角形。
        break;
    } // switch结束

    pDoc->m_basePoint = pg;
    Invalidate( );
    pDoc->m_flagMouseDown = false;

}


void CCP_PolygonPlatformView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	CView::OnMouseMove(nFlags, point);

	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (!pDoc->m_flagSelect)
        return;
    if (!pDoc->m_flagMouseDown)
        return;
    CP_Point2 ps, pg;
    ps.m_x = point.x;
    ps.m_y = point.y;
    CRect r;
    GetClientRect(& r);
    gb_pointConvertFromScreenToGlobal(pg, ps, 
        pDoc->m_scale, pDoc->m_translation, r.right, r.bottom);
    double vx = pg.m_x - pDoc->m_basePoint.m_x;
    double vy = pg.m_y - pDoc->m_basePoint.m_y;

    if (pDoc->m_flagMoveSame)
    {
        gb_movePointIntArray(pDoc->m_a, pDoc->m_flagSelectIDSetInA, vx, vy);
        gb_movePointIntArray(pDoc->m_b, pDoc->m_flagSelectIDSetInB, vx, vy);
        pDoc->m_basePoint = pg;
        Invalidate( );
        return;
    } // if结束

    switch(pDoc->m_flagSelectType)
    {
    case 1: // 点。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_movePoint(pDoc->m_a, pDoc->m_flagSelectID, vx, vy);
        else gb_movePoint(pDoc->m_b, pDoc->m_flagSelectID, vx, vy);
        break;
    case 2: // 环。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_moveLoop(pDoc->m_a, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID, vx, vy);
        else gb_moveLoop(pDoc->m_b, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID, vx, vy);
        break;
    case 3: // 区域。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_moveRegion(pDoc->m_a, pDoc->m_flagSelectRegion, vx, vy);
        else gb_moveRegion(pDoc->m_b, pDoc->m_flagSelectRegion, vx, vy);
        break;
    case 4: // 多边形。
        if (pDoc->m_flagSelectPolygon==0) // A
             gb_movePolygon(pDoc->m_a, vx, vy);
        else gb_movePolygon(pDoc->m_b, vx, vy);
        break;
    case 5: // 剖分三角形。
        break;
    } // switch结束

    pDoc->m_basePoint = pg;
    Invalidate( );

}


void CCP_PolygonPlatformView::OnNewRightInloop()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    bool flagSuccess = false;
    bool flagA = true;
    CRect r;
    GetClientRect(& r);
    double dr = (r.right<r.bottom ? r.right : r.bottom);
    dr /=4;
    int ir = pDoc->m_flagSelectRegion;
    if (!pDoc->m_flagSelect)
    {
        if (pDoc->m_flagBuildA)
             flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_a, pDoc->m_a.m_regionArray.size( )-1,
                pDoc->m_edgeNumber, dr, 0.0, 0.0);
        else
        {
            flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_b, pDoc->m_b.m_regionArray.size( )-1,
                pDoc->m_edgeNumber, dr, 0.0, 0.0);
            flagA = false;
        } // if/else结束
    }
    else
    {
        switch(pDoc->m_flagSelectType)
        {
        case 1: // 点。
        case 4: // 多边形。
            if (pDoc->m_flagSelectPolygon==0)
                 flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_a, pDoc->m_a.m_regionArray.size( )-1,
                    pDoc->m_edgeNumber, dr, 0.0, 0.0);
            else
            {
                flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_b, pDoc->m_b.m_regionArray.size( )-1,
                    pDoc->m_edgeNumber, dr, 0.0, 0.0);
                flagA = false;
            } // if/else结束
            break;
        case 2: // 环。
        case 3: // 区域。
            if (pDoc->m_flagSelectPolygon==0)
                 flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_a, pDoc->m_flagSelectRegion,
                    pDoc->m_edgeNumber, dr, 0.0, 0.0);
            else
            {
                flagSuccess = gb_polygonNewInLoopRegular(pDoc->m_b, pDoc->m_flagSelectRegion,
                    pDoc->m_edgeNumber, dr, 0.0, 0.0);
                flagA = false;
            } // if/else结束
            break;
        } // switch结束
    } // if/else结束
    Invalidate( );

    char s[100];
    sprintf_s(s, 100, "The new inner ring is in polygon %c.", (flagA ? 'A' : 'B'));
    if (flagSuccess)
         mb_statusSetText("The new inner ring is successfully created.", s);
    else mb_statusSetText("没有创建新内环。", s);
}


void CCP_PolygonPlatformView::OnAddOutloop()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagAdd!=0)
    {
        MessageBox("前一个添加操作还没有完成，请继续前一个操作或按鼠标右键结束前一个添加操作。", "操作错误");
    } // if结束
	if (pDoc->m_flagSelect)
    {
        if (pDoc->m_flagSelectPolygon==0)
             pDoc->m_flagAddIDPolygon = 0;
        else pDoc->m_flagAddIDPolygon = 1;
    }
    else
    {
        if (pDoc->m_flagBuildA)
             pDoc->m_flagAddIDPolygon = 0;
        else pDoc->m_flagAddIDPolygon = 1;
    } // if/else结束
    pDoc->m_flagAddPointArray.clear( );
    pDoc->m_flagAdd = 1;
    mb_statusSetText("请按鼠标左键在工作区中添加新外环的点", "请用鼠标右键结束新外环添加操作");
}


void CCP_PolygonPlatformView::OnAddInloop()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagAdd!=0)
    {
        MessageBox("前一个添加操作还没有完成，请继续前一个操作或按鼠标右键结束前一个添加操作。", "操作错误");
    } // if结束
    bool flagSuceess = false;
    CP_Polygon* pn = NULL;
    if (pDoc->m_flagSelect)
    {
        if (pDoc->m_flagSelectPolygon==0)
        {
            pDoc->m_flagAddIDPolygon = 0;
            pn = &(pDoc->m_a);
        }
        else
        {
            pDoc->m_flagAddIDPolygon = 1;
            pn = &(pDoc->m_b);
        } // if/else结束
       switch(pDoc->m_flagSelectType)
        {
        case 1: // 点。
            flagSuceess = gb_findPointInLoop(*pn, pDoc->m_flagAddIDRegion,
                pDoc->m_flagAddIDLoop, pDoc->m_flagAddIDPointInLoop,
                pDoc->m_flagSelectID);
            break;
        case 2: // 环。
            pDoc->m_flagAddIDRegion = pDoc->m_flagSelectRegion;
            pDoc->m_flagAddIDLoop = pDoc->m_flagSelectID;
            flagSuceess = true;
            break;
        case 3: // 区域。
            pDoc->m_flagAddIDRegion = pDoc->m_flagSelectRegion;
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            flagSuceess = true;
            break;
        case 4: // 多边形。
            pDoc->m_flagAddIDRegion = (*pn).m_regionArray.size( )-1;
            if (pDoc->m_flagAddIDRegion<0)
                break;
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            if (pDoc->m_flagAddIDLoop<0)
                break;
            flagSuceess = true;
            break;
        } // switch结束
    }
    else
    {
        if (pDoc->m_flagBuildA)
        {
            pDoc->m_flagAddIDPolygon = 0;
            pn = &(pDoc->m_a);
        }
        else
        {
            pDoc->m_flagAddIDPolygon = 1;
            pn = &(pDoc->m_b);
        } // if/else结束

        pDoc->m_flagAddIDRegion = (*pn).m_regionArray.size( )-1;
        if (pDoc->m_flagAddIDRegion>=0)
        {
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            if (pDoc->m_flagAddIDLoop>=0)
            {
                flagSuceess = true;
            } // if(idL)结束
        } // if(idr)结束
    } // if/else结束
    if (flagSuceess)
    {
        pDoc->m_flagAddPointArray.clear( );
        pDoc->m_flagAdd = 2;
        mb_statusSetText("请按鼠标左键在工作区中添加新内环的点", "请用鼠标右键结束新内环添加操作");
    }
    else
    {
        pDoc->m_flagAdd = 0;
        if (pDoc->m_flagAddIDPolygon==0)
            mb_statusSetText("内环添加操作失败。", "多边形A还没有任何外环。");
        else
            mb_statusSetText("内环添加操作失败。", "请给多边形B增加外环，再执行本操作。");
    } // if/else结束
}


void CCP_PolygonPlatformView::OnAddPoint()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (pDoc->m_flagAdd!=0)
    {
        MessageBox("前一个添加操作还没有完成，请继续前一个操作或按鼠标右键结束前一个添加操作。", "操作错误");
        return;
    } // if结束

    bool flagSuceess = false;
    CP_Polygon* pn = NULL;
	if (pDoc->m_flagSelect)
    {
        if (pDoc->m_flagSelectPolygon==0)
        {
            pDoc->m_flagAddIDPolygon = 0;
            pn = &(pDoc->m_a);
        }
        else
        {
            pDoc->m_flagAddIDPolygon = 1;
            pn = &(pDoc->m_b);
        } // if/else结束

        switch(pDoc->m_flagSelectType)
        {
        case 1: // 点。
            flagSuceess = gb_findPointInLoop(*pn, pDoc->m_flagAddIDRegion,
                pDoc->m_flagAddIDLoop, pDoc->m_flagAddIDPointInLoop,
                pDoc->m_flagSelectID);
            break;
        case 2: // 环。
            pDoc->m_flagAddIDRegion = pDoc->m_flagSelectRegion;
            pDoc->m_flagAddIDLoop = pDoc->m_flagSelectID;
            pDoc->m_flagAddIDPointInLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray.size( )-1; 
            if (pDoc->m_flagAddIDPointInLoop<0)
                break;
            flagSuceess = true;
            break;
        case 3: // 区域。
            pDoc->m_flagAddIDRegion = pDoc->m_flagSelectRegion;
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            pDoc->m_flagAddIDPointInLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray.size( )-1; 
            if (pDoc->m_flagAddIDPointInLoop<0)
                break;
            flagSuceess = true;
            break;
        case 4: // 多边形。
            pDoc->m_flagAddIDRegion = (*pn).m_regionArray.size( )-1;
            if (pDoc->m_flagAddIDRegion<0)
                break;
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            if (pDoc->m_flagAddIDLoop<0)
                break;
            pDoc->m_flagAddIDPointInLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray.size( )-1; 
            if (pDoc->m_flagAddIDPointInLoop<0)
                break;
            flagSuceess = true;
            break;
        } // switch结束
    }
    else
    {
        if (pDoc->m_flagBuildA)
        {
            pDoc->m_flagAddIDPolygon = 0;
            pn = &(pDoc->m_a);
        }
        else
        {
            pDoc->m_flagAddIDPolygon = 1;
            pn = &(pDoc->m_b);
        } // if/else结束

        pDoc->m_flagAddIDRegion = (*pn).m_regionArray.size( )-1;
        if (pDoc->m_flagAddIDRegion>=0)
        {
            pDoc->m_flagAddIDLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray.size( )-1;
            if (pDoc->m_flagAddIDLoop>=0)
            {
                pDoc->m_flagAddIDPointInLoop = (*pn).m_regionArray[pDoc->m_flagAddIDRegion].m_loopArray[pDoc->m_flagAddIDLoop].m_pointIDArray.size( )-1; 
                if (pDoc->m_flagAddIDPointInLoop>=0)
                {
                    flagSuceess = true;
                } // if(idLv)结束
            } // if(idL)结束
        } // if(idr)结束
    } // if/else结束
    if (flagSuceess)
    {
        pDoc->m_flagAdd = 3;
        mb_statusSetText("用鼠标左键在工作区中添加点", "用鼠标右键结束点添加操作");
    }
    else
    {
        pDoc->m_flagAdd = 0;
        if (pDoc->m_flagAddIDPolygon==0)
            mb_statusSetText("点添加操作失败。", "多边形A还没有任何外环。");
        else
            mb_statusSetText("点添加操作失败。", "请给多边形B增加外环，再执行本操作。");
        } // if/else结束
}


void CCP_PolygonPlatformView::OnDelete()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    if (!pDoc->m_flagSelect)
    {
        MessageBox("还没有拾取图形，无法删除。", "无效操作");
		return;
    } // if结束
    if (pDoc->m_flagAdd!=0)
    {
        MessageBox("添加图形的操作还没有结束，无法删除。", "无效操作");
		return;
    } // if结束
    CP_Polygon* pn;
    if (pDoc->m_flagSelectPolygon==0)
         pn = &(pDoc->m_a);
    else pn = &(pDoc->m_b);
    switch(pDoc->m_flagSelectType)
    {
    case 1: // 点。
        gb_removePoint(*pn, pDoc->m_flagSelectID);
        break;
    case 2: // 环。
        gb_removeLoop(*pn, pDoc->m_flagSelectRegion, pDoc->m_flagSelectID);
        break;
    case 3: // 区域。
        gb_removeRegion(*pn, pDoc->m_flagSelectRegion);
        break;
    case 4: // 多边形。
        pn->mb_clear( );
        break;
    } // switch结束
    pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateMoveSame(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pCmdUI->SetCheck(pDoc->m_flagMoveSame);

}


void CCP_PolygonPlatformView::OnMoveSame()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_flagMoveSame ^= true;
    if (!(pDoc->m_flagMoveSame))
    {
        pDoc->m_flagSelectIDSetInA.clear( );
        pDoc->m_flagSelectIDSetInB.clear( );
    } // if结束
}


void CCP_PolygonPlatformView::OnUpdateViewA(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pCmdUI->SetCheck(pDoc->m_flagShowA);

}


void CCP_PolygonPlatformView::OnViewA()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_flagShowA ^= true;

    if (!(pDoc->m_flagShowA))
        if (pDoc->m_flagSelectPolygon==0)
            pDoc->m_flagSelect = false;

    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateViewB(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pCmdUI->SetCheck(pDoc->m_flagShowB);

}


void CCP_PolygonPlatformView::OnViewB()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_flagShowB ^= true;
    if (!(pDoc->m_flagShowB))
        if (pDoc->m_flagSelectPolygon!=0)
            pDoc->m_flagSelect = false;
    Invalidate(); // 刷新

}


void CCP_PolygonPlatformView::OnUpdateViewPointId(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pCmdUI->SetCheck(pDoc->m_flagShowPointID);

}


void CCP_PolygonPlatformView::OnViewPointId()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_flagShowPointID ^= true;
    Invalidate(); // 刷新

}

void CCP_PolygonPlatformView::OnCheck()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

    if (pDoc->m_a.m_regionArray.size() > 0) {
        if (gb_checkPolygon(pDoc->m_a)) {
            MessageBox("general polygon A is legal");
        }
        else
            MessageBox("General polygon A is illegal");
    }
    if (pDoc->m_b.m_regionArray.size() > 0) {
        if (gb_checkPolygon(pDoc->m_b)) {
            MessageBox("General polygon B is legal");
        }
        else
            MessageBox("General polygon B is illegal");
    }
}

void CCP_PolygonPlatformView::OnUpdateViewResult(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
	pCmdUI->SetCheck(pDoc->m_showBsptree);
}

void CCP_PolygonPlatformView::OnViewResult()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
    pDoc->m_showBsptree ^= true;
    Invalidate();
}

void CCP_PolygonPlatformView::OnPolygonUnion()
{
    printf("OnPolygonUnion\n");
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
	clock_t start, median1, median2, end;
	start = clock();

	CP_BSPNode *a_tree = gb_buildPolygonBSPTree(pDoc->m_a);
	CP_BSPNode *b_tree = gb_buildPolygonBSPTree(pDoc->m_b);

	median1 = clock();
    
    printf("\t OnPolygonUnion - merging A and B BSPs\n");
	if(a_tree == NULL && b_tree == NULL){
		return;
	}
	else if(a_tree == NULL){
		pDoc->m_bspTree = b_tree;
	}
	else if(b_tree == NULL){
		pDoc->m_bspTree = a_tree;
	}
    else {
        pDoc->m_bspTree = gb_mergeBSPTree_root(a_tree, b_tree, CP_BSPOp::UNION);
    }

	median2 = clock();

	gb_generateCellPolygons(pDoc->m_bspTree);
	gb_generateBSPTreeFaces(pDoc->m_bspTree);

	end = clock();

    debugBsptree(pDoc->m_bspTree);

	m_buildTime = median1 - start;
	m_generateEdgeTime = end - median2;
	m_mergeTime = median2 - median1;

    pDoc->m_showBsptree = true;
	Invalidate();
}

void CCP_PolygonPlatformView::OnPolygonIntersection()
{
    printf("OnPolygonIntersection\n");
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	clock_t start, median1, median2, end;
	start = clock();
	CP_BSPNode *a_tree = gb_buildPolygonBSPTree(pDoc->m_a);
	CP_BSPNode *b_tree = gb_buildPolygonBSPTree(pDoc->m_b);
	median1 = clock();
	if(a_tree == NULL && b_tree == NULL){
		return;
	}
	else if(a_tree == NULL){
		pDoc->m_bspTree = b_tree;
	}
	else if(b_tree == NULL){
		pDoc->m_bspTree = a_tree;
	}
	else 
		pDoc->m_bspTree = gb_mergeBSPTree_root(a_tree, b_tree, CP_BSPOp::INTERSECTION);

	median2 = clock();
	gb_generateCellPolygons(pDoc->m_bspTree);
	gb_generateBSPTreeFaces(pDoc->m_bspTree);
	debugBsptree(pDoc->m_bspTree);
	
	end = clock();
	m_buildTime = median1 - start;
	m_generateEdgeTime = end - median2;
	m_mergeTime = median2 - median1;

    pDoc->m_showBsptree = true;
	Invalidate();
}

void CCP_PolygonPlatformView::OnPolygonAB()
{
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
	clock_t start, median1, median2, end;
	start = clock();
	CP_BSPNode *a_tree = gb_buildPolygonBSPTree(pDoc->m_a);
	CP_BSPNode *b_tree = gb_buildPolygonBSPTree(pDoc->m_b);
	median1 = clock();

	if(a_tree == NULL && b_tree == NULL){
		return;
	}
	else if(a_tree == NULL){
		pDoc->m_bspTree = b_tree;
	}
	else if(b_tree == NULL){
		pDoc->m_bspTree = a_tree;
	}
	else 
		pDoc->m_bspTree = gb_mergeBSPTree_root(a_tree, b_tree, CP_BSPOp::SUBTRACTION);

	median2 = clock();
	gb_generateCellPolygons(pDoc->m_bspTree);
	gb_generateBSPTreeFaces(pDoc->m_bspTree);
	debugBsptree(pDoc->m_bspTree);
	
	end = clock();
	m_buildTime = median1 - start;
	m_generateEdgeTime = end - median2;
	m_mergeTime = median2 - median1;
    pDoc->m_showBsptree = true;
    Invalidate();
}

void CCP_PolygonPlatformView::OnPolygonBA()
{
	// TODO: 在此添加命令处理程序代码
	CCP_PolygonPlatformDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
	clock_t start, median1, median2, end;
	start = clock();
	CP_BSPNode *a_tree = gb_buildPolygonBSPTree(pDoc->m_a);
	CP_BSPNode *b_tree = gb_buildPolygonBSPTree(pDoc->m_b);
	median1 = clock();
	if(a_tree == NULL && b_tree == NULL){
		return;
	}
	else if(a_tree == NULL){
		pDoc->m_bspTree = b_tree;
	}
	else if(b_tree == NULL){
		pDoc->m_bspTree = a_tree;
	}
	else 
		pDoc->m_bspTree = gb_mergeBSPTree_root(b_tree, a_tree, CP_BSPOp::SUBTRACTION);
	
	
	median2 = clock();
	gb_generateCellPolygons(pDoc->m_bspTree);
	gb_generateBSPTreeFaces(pDoc->m_bspTree);
	debugBsptree(pDoc->m_bspTree);
	
	end = clock();
	m_buildTime = median1 - start;
	m_generateEdgeTime = end - median2;
	m_mergeTime = median2 - median1;
    pDoc->m_showBsptree = true;
    Invalidate();
}

BOOL CCP_PolygonPlatformView::OnEraseBkgnd(CDC* pDC){return CView::OnEraseBkgnd(pDC);}
