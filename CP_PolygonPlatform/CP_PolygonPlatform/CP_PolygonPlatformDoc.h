#pragma once

#include "CP_Polygon.h"

class CCP_PolygonPlatformDoc : public CDocument
{
protected: // 仅从序列化创建
	CCP_PolygonPlatformDoc();
	DECLARE_DYNCREATE(CCP_PolygonPlatformDoc)
public:
    virtual ~CCP_PolygonPlatformDoc();

public:
	// //////////////////////////////////////////////////////////////
    // 下面变量将被保存
    CP_Polygon m_a, m_b;
    double m_tolerance; // 位置容差
    double m_scale; // 缩放比例
    CP_Point2 m_translation; // 坐标平移量

	// //////////////////////////////////////////////////////////////
    // 下面变量不被保存
    CP_Polygon m_result;
    //CP_TriagleMesh m_triagleMesh;
    bool m_flagBuildA; // true: A; false B。
    bool m_flagSelect; // true: 已经拾取到; false: 没有拾取任何东西。
    int  m_flagSelectType;
    // 0: 没有设置; 1: 点; 2: 环; 3: 区域; 4: 多边形; 5: 剖分三角形。

    int  m_flagSelectPolygon, m_flagSelectRegion, m_flagSelectID;
    // 定位拾取的内容
    // m_flagSelectPolygon(0: A; 1: B)

    VT_IntArray m_flagSelectIDSetInA, m_flagSelectIDSetInB;
    bool m_flagShowSelect; // true: 只显示选择集; false: 正常显示。
    int  m_edgeNumber; // 正多边形的边数。
    int  m_flagAdd; // 0: 没有设置; 1: 外环; 2: 内环; 3: 点。
    int  m_flagAddIDPolygon, m_flagAddIDRegion, m_flagAddIDLoop, m_flagAddIDPointInLoop;
    VT_PointArray m_flagAddPointArray;
	bool m_flagShowA, m_flagShowB, m_flagShowPointID;
     // true: 显示; false: 不显示。

    bool m_flagMoveSame; // true: 移动重合的点; false: 重合点分开移动。

 // 下面是一些临时性变量
    CP_Point2 m_basePoint;
    bool m_flagMouseDown; // true: 按下鼠标左键; false: 放开鼠标左键
	bool m_showBsptree;
	CP_BSPNode *m_bspTree;

public:
	void mb_initData( );

public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif
protected:
	DECLARE_MESSAGE_MAP()
};

extern void gb_SerializePolygon(CArchive& ar, CP_Polygon& p);
