#pragma once

class CCP_PolygonPlatformView : public CView
{
protected:
	CCP_PolygonPlatformView();
	DECLARE_DYNCREATE(CCP_PolygonPlatformView)
public:
	virtual ~CCP_PolygonPlatformView();

public:
	CCP_PolygonPlatformDoc* GetDocument() const;
	double m_buildTime;
	double m_mergeTime;
	double m_generateEdgeTime;

public:
	void mb_statusSetText(char* s1, char* s2);

public:
	virtual void OnDraw(CDC* pDC);
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnUpdateComboAorb(CCmdUI *pCmdUI);
	afx_msg void OnComboAorb();
	afx_msg void OnEdgeNumber();
	afx_msg void OnTolerance();
	afx_msg void OnViewStandard();
	afx_msg void OnViewFit();
	afx_msg void OnUpdateSelectPoint(CCmdUI *pCmdUI);
	afx_msg void OnSelectPoint();
	afx_msg void OnUpdateSelectLoop(CCmdUI *pCmdUI);
	afx_msg void OnSelectLoop();
	afx_msg void OnUpdateSelectRegion(CCmdUI *pCmdUI);
	afx_msg void OnSelectRegion();
	afx_msg void OnUpdateSelectPolygon(CCmdUI *pCmdUI);
	afx_msg void OnSelectPolygon();
	afx_msg void OnUpdateSelectTriangle(CCmdUI *pCmdUI);
	afx_msg void OnSelectTriangle();
	afx_msg void OnUpdateSelectOnly(CCmdUI *pCmdUI);
	afx_msg void OnSelectOnly();
	afx_msg void OnNewRightOutloop();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnNewRightInloop();
	afx_msg void OnAddOutloop();
	afx_msg void OnAddInloop();
	afx_msg void OnAddPoint();
	afx_msg void OnDelete();
	afx_msg void OnUpdateMoveSame(CCmdUI *pCmdUI);
	afx_msg void OnMoveSame();
	afx_msg void OnUpdateViewA(CCmdUI *pCmdUI);
	afx_msg void OnViewA();
	afx_msg void OnUpdateViewB(CCmdUI *pCmdUI);
	afx_msg void OnViewB();
	afx_msg void OnUpdateViewPointId(CCmdUI *pCmdUI);
	afx_msg void OnViewPointId();
	afx_msg void OnCheck();
	afx_msg void OnPolygonUnion();
	afx_msg void OnUpdateViewResult(CCmdUI *pCmdUI);
	afx_msg void OnViewResult();
	afx_msg void OnPolygonIntersection();
	afx_msg void OnPolygonAB();
	afx_msg void OnPolygonBA();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
};

extern void gb_drawLoop(CDC* pDC, CP_Loop& p, 
                 double scale, CP_Point2 translation, int screenX, int screenY, 
                 int r, int g, int b, int size);
extern void gb_drawPointArrayLine(CDC* pDC, VT_PointArray& pa,
                               double scale, CP_Point2 translation, int screenX, int screenY,
                               int r, int g, int b, int size);
extern void gb_drawPointArrayPoint(CDC* pDC, VT_PointArray& pa,
                               double scale, CP_Point2 translation, int screenX, int screenY,
                               int r, int g, int b, int size);
extern void gb_drawPointGlobal(CDC* pDC, CP_Point2 pointGlobal,
                               double scale, CP_Point2 translation, int screenX, int screenY,
                               int r, int g, int b, int size);
extern void gb_drawPointScreen(CDC* pDC, int x, int y,
                               int r, int g, int b, int size);
extern void gb_drawPolygonLoop(CDC* pDC, CP_Polygon& p,
                        double scale, CP_Point2 translation, int screenX, int screenY,
                        int outR, int outG, int outB, 
                        int inR,  int inG,  int inB, 
                        int size);
extern void gb_drawPolygonPoint(CDC* pDC, CP_Polygon& p,
                                double scale, CP_Point2 translation, int screenX, int screenY,
                                int r, int g, int b, int size);
extern void gb_drawPolygonPointID(CDC* pDC, CP_Polygon& p,
                                double scale, CP_Point2 translation, int screenX, int screenY,
                                int r, int g, int b);
extern void gb_drawBspTree(CDC* pDC, CP_BSPNode* tree,
        double scale, CP_Point2 translation, int screenX, int screenY,
        int r, int g, int b, int size);

extern void gb_drawBspNode(CDC* pDC, CP_BSPNode* tree,
        double scale, CP_Point2 translation, int screenX, int screenY,
        int r, int g, int b, int size);

#ifndef _DEBUG
inline CCP_PolygonPlatformDoc* CCP_PolygonPlatformView::GetDocument() const
   { return reinterpret_cast<CCP_PolygonPlatformDoc*>(m_pDocument); }
#endif

