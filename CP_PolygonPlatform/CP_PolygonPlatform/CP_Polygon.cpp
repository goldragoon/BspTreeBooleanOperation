#include "stdafx.h"
#include "CP_Polygon.h"
#include <cmath>
#include <fstream>
using namespace std;
vector<CP_BSPNode*> CP_BSPNodeList;
vector<CP_Partition*> CP_PartitionList;
vector<CP_Point> CP_PointList;
void gb_distanceMinPointLoop(double&d, int& idRegion, int& idLoop,
                             CP_Point& pt, CP_Polygon& pn)
{ 
    d = 0.0;
    idRegion = -1;
    idLoop = -1;
    int nr = pn.m_regionArray.size( );
    int i, j, k, nl, nv, v1, v2;
    double dt;
    for (i=0; i<nr; i++)
    {
        nl = pn.m_regionArray[i].m_loopArray.size( );
        for (j=0; j<nl; j++)
        {
            nv = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( );
           for (k=0; k<nv; k++)
            {
                v1 = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[k];
                if (k==nv-1)
                    v2 = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[0];
                else v2 = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[k+1];
                dt = gb_distancePointSegment(pt, pn.m_pointArray[v1], pn.m_pointArray[v2]);
                if ((idLoop==-1) || (d>dt))
                {
                    d = dt;
                    idRegion = i;
                    idLoop = j;
                }
            }
        }
    }
}

void gb_distanceMinPointPolygon(double&d,int& id,CP_Point& pt,CP_Polygon& pn)
{
    d = 0.0;
    id = -1;
    int n = pn.m_pointArray.size( );
    if (n<=0)
        return;
    d = gb_distancePointPoint(pt, pn.m_pointArray[0]);
    id = 0;
    int i;
    double dt;
    for (i=1; i<n; i++)
    {
        dt = gb_distancePointPoint(pt, pn.m_pointArray[i]);
        if (dt<d)
        {
            d = dt;
            id = i;
        } // if结束
    } // for结束
} // 函数gb_distanceMinPointPolygon结束

double gb_distancePointPoint(CP_Point& p1, CP_Point& p2)
{
    double dx = p1.m_x - p2.m_x;
    double dy = p1.m_y - p2.m_y;
    double d2 = dx*dx + dy*dy;
    double d = sqrt(d2);
    return d;
} // 函数gb_distancePointPoint结束

double gb_distancePointSegment(CP_Point& pt, CP_Point& p1, CP_Point& p2)
{
    double dx0 = p2.m_x - p1.m_x;
    double dy0 = p2.m_y - p1.m_y;
    double dx1 = pt.m_x - p1.m_x;
    double dy1 = pt.m_y - p1.m_y;
    double dx2 = pt.m_x - p2.m_x;
    double dy2 = pt.m_y - p2.m_y;
    double d1 = dx1*dx1 + dy1*dy1;
    double d2 = dx2*dx2 + dy2*dy2;
    double d01 = dx1*dx0 + dy1*dy0;
    double d02 =-dx2*dx0 - dy2*dy0;
    double d, d0;
    if ((d01>0) && (d02>0))
    {
        d0 = dx0*dx0 + dy0*dy0;
        d = d01*d01/d0; // 如果计算溢出，如何处理?
        d = d1 - d;
        d = sqrt(d);
        return d;
    } // if结束
    if (d1>d2)
        d = d2;
    else d = d1;
    d = sqrt(d);
    return d;
} // 函数gb_distancePointPoint结束

void gb_getIntArrayPointInPolygon(VT_IntArray& vi, CP_Polygon& pn, CP_Point& p, double eT)
{
    int i, n;
    double d;
    n = pn.m_pointArray.size( );
    for (i=0; i<n; i++)
    {
        d = gb_distancePointPoint(p, pn.m_pointArray[i]);
        if (d<=eT)
        {
            vi[i] = i;
        } // if结束
    } // for(i)结束
} // 函数gb_getIntArrayPointInPolygon结束

bool gb_findPointInLoop(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, int pointInPolygon)
{
    idRegion = 0;
    idLoop = 0;
    idPointInLoop = 0;
    int nr, nL, nv;
    int i, j, k;
    nr = pn.m_regionArray.size( );
    for (i=0; i<nr; i++)
    {
        nL = pn.m_regionArray[i].m_loopArray.size( );
        for (j=0; j<nL; j++)
        {
            nv = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( );
           for (k=0; k<nv; k++)
            {
                if (pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[k]==pointInPolygon)
                {
                    idRegion = i;
                    idLoop = j;
                    idPointInLoop = k;
                    return true;
                } // if结束
            } // for(nv)结束
        } // for(nL)结束
    } // for(nr)结束
    return false;
} // 函数gb_findPointInLoop结束

// 这里假设所有的条件都成立，即函数内部不判断输入的合法性
void gb_insertPointInPolygon(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, CP_Point& newPoint)
{
    int nv = pn.m_pointArray.size( );
    pn.m_pointArray.push_back(newPoint);
    pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.insert(
        pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.begin( )+idPointInLoop+1,
        nv);
} // 函数gb_findPointInLoop结束

void gb_intArrayInit(VT_IntArray& vi, int data)
{
    int n = vi.size( );
    int i;
    for (i=0; i<n; i++)
        vi[i] = data;
} // 函数gb_intArrayInit结束

void gb_intArrayInitLoop(VT_IntArray& vi, CP_Polygon& pn, int idRegion, int idLoop, double eT)
{
    int i, v;
    int n = pn.m_pointArray.size( );
    vi.resize(n);
    n = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.size( );
    for (i=0; i<n; i++)
    {
        v = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray[i];
        vi[v] = v;
    } // for结束
    gb_intArrayInitPointSame(vi, pn, eT);
} // 函数gb_intArrayInitLoop结束

void gb_intArrayInitPoint(VT_IntArray& vi, CP_Polygon& pn, int v, double eT)
{
    int n = pn.m_pointArray.size( );
    if (n<=0)
    {
        vi.clear( );
        return;
    } // if结束
    vi.resize(n);
    int i;
    double d;
 for (i=0; i<n; i++)
    {
        if (i==v)
            vi[i] = i;
        else
        {
            d = gb_distancePointPoint(pn.m_pointArray[i], pn.m_pointArray[v]);
            if (d <= eT)
                vi[i] = i;
            else vi[i] = -1;
        } // if/else结束
    } // for结束
} // 函数gb_intArrayInitPoint结束

void gb_intArrayInitPointSame(VT_IntArray& vi, CP_Polygon& pn, double eT)
{
    int i, j, n;
    double d;
    n = vi.size( );
    if (n<=0)
        return;
    for (i=0; i<n; i++)
    {
        if (vi[i]>=0)
        {
            for (j=0; j<n; j++)
            {
                if (vi[j]<0)
                {
                    d = gb_distancePointPoint(pn.m_pointArray[i], pn.m_pointArray[j]);
                    if (d<=eT)
                        vi[j] = j;
                } // if结束
            } // for(j)结束
        } // if结束
    } // for(i)结束
} // 函数gb_intArrayInitPointSame结束

void gb_intArrayInitPolygon(VT_IntArray& vi, CP_Polygon& pn)
{
    int i;
    int n = pn.m_pointArray.size( );
    vi.resize(n);
    for (i=0; i<n; i++)
        vi[i] = i;
} // 函数gb_intArrayInitPolygon结束

void gb_intArrayInitPolygonSamePoint(VT_IntArray& vr, CP_Polygon& pr, VT_IntArray& vs, CP_Polygon& ps, double eT)
{
    int i, j;
    int n0, n1;
    double da;
    n1 = pr.m_pointArray.size( );
    if (n1<=0)
    {
        vr.clear( );
        return;
    } // if结束
    vr.resize(n1);
    gb_intArrayInit(vr, -1);
    n0 = ps.m_pointArray.size( );
    for (i=0; i<n0; i++)
    {
        if (vs[i]<0)
            continue;
        for (j=0; j<n1; j++)
        {
            if (vr[j]<0)
            {
                da = gb_distancePointPoint(ps.m_pointArray[i], pr.m_pointArray[j]);
                if (da <= eT)
                    vr[j] = j;
            } // if结束
        } // for(j)结束
    } // for(i)结束
} // 函数gb_intArrayInitPolygonSamePoint结束

void gb_intArrayInitRegion(VT_IntArray& vi, CP_Polygon& pn, int idRegion, double eT)
{
    int i, j, nr, v;
    int n = pn.m_pointArray.size( );
    vi.resize(n);
    nr = pn.m_regionArray[idRegion].m_loopArray.size( );
    for (i=0; i<nr; i++)
    {
        n = pn.m_regionArray[idRegion].m_loopArray[i].m_pointIDArray.size( );
        for (j=0; j<n; j++)
        {
            v = pn.m_regionArray[idRegion].m_loopArray[i].m_pointIDArray[j];
            vi[v] = v;
        } // for(j)结束
    } // for(i)结束
    gb_intArrayInitPointSame(vi, pn, eT);
} // 函数gb_intArrayInitRegion结束

void gb_moveLoop(CP_Polygon& pn, int idRegion, int idLoop, double vx, double vy)
{
    int nr, nL, nv;
    int i, id;
    nr = pn.m_regionArray.size( );
    if ((idRegion<0) || (idRegion>=nr))
        return;
    nL = pn.m_regionArray[idRegion].m_loopArray.size( );
    if ((idLoop<0) || (idLoop>=nL))
        return;
    nv = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.size( );
    for (i=0; i<nv; i++)
    {
        id = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray[i];
        pn.m_pointArray[id].m_x += vx;
        pn.m_pointArray[id].m_y += vy;
    } // for结束
} // 函数gb_moveLoop结束

void gb_movePoint(CP_Polygon& pn, int id, double vx, double vy)
{
    int n = pn.m_pointArray.size( );
    if ((id<0) || (id>=n))
        return;
    pn.m_pointArray[id].m_x += vx;
    pn.m_pointArray[id].m_y += vy;
} // 函数gb_movePoint结束

void gb_movePointIntArray(CP_Polygon& pn, VT_IntArray& vi, double vx, double vy)
{
    int n = vi.size( );
    int i;
    for (i=0; i<n; i++)
        gb_movePoint(pn, vi[i], vx, vy);
} // 函数gb_movePoint结束

void gb_movePolygon(CP_Polygon& pn, double vx, double vy)
{
    int n = pn.m_pointArray.size( );
    int i;
    for (i=0; i<n; i++)
    {
        pn.m_pointArray[i].m_x += vx;
        pn.m_pointArray[i].m_y += vy;
    } // for结束
} // 函数gb_movePolygon结束

void gb_moveRegion(CP_Polygon& pn, int idRegion, double vx, double vy)
{
    int nr, nL, nv;
    int i, j, k, id;
    nr = pn.m_regionArray.size( );
    if ((idRegion<0) || (idRegion>=nr))
        return;
    i = idRegion;
    nL = pn.m_regionArray[i].m_loopArray.size( );
    for (j=0; j<nL; j++)
    {
        nv = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray.size( );
        for (k=0; k<nv; k++)
        {
            id = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[k];
            pn.m_pointArray[id].m_x += vx;
            pn.m_pointArray[id].m_y += vy;
        } // for结束
    } // for结束
} // 函数gb_moveRegion结束

// 将在全局坐标系下的点转换成为在屏幕坐标下的点
// result:      输出的在屏幕坐标下的点;
// pointGlobal: 输入的在全局坐标系下的点;
// scale:       输入的比例因子;
// translation: 输入的平移坐标值。
void gb_pointConvertFromGlobalToScreen(CP_Point& result, CP_Point pointGlobal, double scale, CP_Point translation, int screenX, int screenY)
{
    result.m_x=(pointGlobal.m_x-translation.m_x)*scale;
    result.m_y=(pointGlobal.m_y-translation.m_y)*scale;
    result.m_x+= (screenX/2);
    result.m_y=screenY/2-result.m_y;
} // 函数PointConvertFromGlobalToScreen结束

// 将在屏幕坐标下的点转换成为在全局坐标系下的点
// result:      输出的在全局坐标系下的点;
// pointScreen: 输入的在屏幕坐标系下的点;
// scale:       输入的比例因子;
// translation: 输入的平移坐标值。
void gb_pointConvertFromScreenToGlobal(CP_Point& result, CP_Point pointScreen, double scale, CP_Point translation, int screenX, int screenY)
{
    result.m_x=pointScreen.m_x - screenX/2;
    result.m_y=screenY/2-pointScreen.m_y;
    result.m_x=result.m_x/scale+translation.m_x;
    result.m_y=result.m_y/scale+translation.m_y;
} // 函数gb_PointConvertFromScreenToGlobal结束

// 给多边形p增加新的内环，该内环是外接圆半径为r的正n边形。
bool gb_polygonNewInLoopRegular(CP_Polygon& p, int idRegion, int n, double r, double cx, double cy)
{
    if (n<3)
        return false;
    int nr = p.m_regionArray.size( );
    if ((idRegion<0)||(idRegion>=nr))
        return false;
    int nL = p.m_regionArray[idRegion].m_loopArray.size( );
    if (nL<=0)
        return false;
    p.m_regionArray[idRegion].m_loopArray.resize(nL+1);
    int s = p.m_pointArray.size( );
    int t = s + n;
    int i, k;
    p.m_pointArray.resize(t);
    double da = DOUBLE_PI / n;
    double d = 0.0;
    for (i=s; i<t; i++, d+=da)
    {
        p.m_pointArray[i].m_x = cx + r*cos(d);
        p.m_pointArray[i].m_y = cy + r*sin(d);
    } // for结束
    p.m_regionArray[idRegion].m_loopArray[nL].m_polygon = &p;
    p.m_regionArray[idRegion].m_loopArray[nL].m_regionIDinPolygon = idRegion;
    p.m_regionArray[idRegion].m_loopArray[nL].m_loopIDinRegion = nL;
    p.m_regionArray[idRegion].m_loopArray[nL].m_pointIDArray.resize(n);
    for (i=0, k= t-1; i<n; i++, k--)
    {
        p.m_regionArray[idRegion].m_loopArray[nL].m_pointIDArray[i] = k;
    } // for结束
    return true;
} // 函数gb_polygonNewInLoopRegular结束

// 给多边形p增加新的外环，该外环是外接圆半径为r的正n边形。
void gb_polygonNewOutLoopRegular(CP_Polygon& p, int n, double r, double cx, double cy)
{
    if (n<3)
        return;
    int s = p.m_pointArray.size( );
    int t = s + n;
    int i, k;
    p.m_pointArray.resize(t);
    double da = DOUBLE_PI / n;
    double d = 0.0;
    for (i=s; i<t; i++, d+=da)
    {
        p.m_pointArray[i].m_x = cx + r*cos(d);
        p.m_pointArray[i].m_y = cy + r*sin(d);
    } // for结束
    int rs = p.m_regionArray.size( );
    p.m_regionArray.resize(rs+1);
    p.m_regionArray[rs].m_polygon = & p;
    p.m_regionArray[rs].m_regionIDinPolygon = rs;
    p.m_regionArray[rs].m_loopArray.resize(1);
    p.m_regionArray[rs].m_loopArray[0].m_polygon = &p;
    p.m_regionArray[rs].m_loopArray[0].m_regionIDinPolygon = rs;
    p.m_regionArray[rs].m_loopArray[0].m_loopIDinRegion = 0;
    p.m_regionArray[rs].m_loopArray[0].m_pointIDArray.resize(n);
    for (i=0, k= s; i<n; i++, k++)
    {
        p.m_regionArray[rs].m_loopArray[0].m_pointIDArray[i] = k;
    } // for结束
} // 函数gb_polygonNewOutLoopRegular结束

bool gb_removeLoop(CP_Polygon& pn, int idRegion, int idLoop)
{
    int nL, nLv, iLv, v;
    nL = pn.m_regionArray[idRegion].m_loopArray.size( );
    if ((idLoop==0) || (nL<2))
        return(gb_removeRegion(pn, idRegion));
    nLv = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.size( );
    for (iLv=0; iLv<nLv; iLv++)
    {
        v = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray[iLv];
        pn.m_pointArray.erase(pn.m_pointArray.begin( )+v);
        gb_subtractOneAboveID(pn, v);
    } // for(iLv)结束
    pn.m_regionArray[idRegion].m_loopArray.erase(
        pn.m_regionArray[idRegion].m_loopArray.begin( )+idLoop);
    return true;
} // 函数gb_removeLoop结束

bool gb_removePoint(CP_Polygon& pn, int id)
{
    int ir, iL, iLv, nLv;
    bool rf = gb_findPointInLoop(pn, ir, iL, iLv, id);
    if (!rf)
        return false;
    nLv = pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.size( );
    if (nLv<4) // 删除整个环
        return (gb_removeLoop(pn, ir, iL));
    pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.erase(
        pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.begin( )+iLv);
    pn.m_pointArray.erase(pn.m_pointArray.begin( )+id);
    gb_subtractOneAboveID(pn, id);
    return true;
} // 函数gb_removePoint结束

bool gb_removeRegion(CP_Polygon& pn, int idRegion)
{
    int nr, nL, nLv, iL, iLv, v;
    nr = pn.m_regionArray.size( );
    if (nr<2)
    {
        pn.mb_clear( );
        return true;
    } // if结束`
    nL = pn.m_regionArray[idRegion].m_loopArray.size( );
    for (iL=0; iL<nL; iL++)
    {
        nLv =  pn.m_regionArray[idRegion].m_loopArray[iL].m_pointIDArray.size( );
        for (iLv=0; iLv<nLv; iLv++)
        {
            v = pn.m_regionArray[idRegion].m_loopArray[iL].m_pointIDArray[iLv];
            pn.m_pointArray.erase(pn.m_pointArray.begin( )+v);
            gb_subtractOneAboveID(pn, v);
        } // for(iLv)结束
    } // for(iL)结束
    pn.m_regionArray.erase(pn.m_regionArray.begin( )+idRegion);
    return true;
} // 函数gb_removeRegion结束

void gb_subtractOneAboveID(CP_Polygon& pn, int id)
{
    int nr = pn.m_regionArray.size( );
    int nL, nLv, ir, iL, iLv;
    for (ir=0; ir<nr; ir++)
    {
        nL = pn.m_regionArray[ir].m_loopArray.size( );
        for (iL=0; iL<nL; iL++)
        {
            nLv =  pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.size( );
            for (iLv=0; iLv<nLv; iLv++)
            {
                if (pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray[iLv]>=id)
                    pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray[iLv]--;
            }
        }
    }
}

//my code
//合法性检查
bool gb_checkPolygon(CP_Polygon& pn)
{
	//检查单个区域
	int nr = pn.m_regionArray.size();
	for(int iR = 0; iR < nr; iR++)
	{
		if(!gb_checkRegion(pn.m_regionArray[iR]))
		{
			return false;
		}
	}

	//边相交
	for(int i = 0; i < nr - 1; i++){
		for(int j = i + 1; j < nr; j++){
			if(gb_checkRegionCrossRegion(pn.m_regionArray[i], pn.m_regionArray[j]))
				return false;
		}
	}

	//检查区域之间是否有区域重合
	for(int i = 0; i < nr - 1; i++){
		for(int j = i + 1; j < nr; j++){
			CP_BSPNode *a = gb_buildRegionBSPTree(pn.m_regionArray[i]);
			CP_BSPNode *b = gb_buildRegionBSPTree(pn.m_regionArray[j]);
			if(gb_tree1OverlapWithTree2(a, b))
				return false;
		}
	}

	return true;
}

bool gb_checkRegion(CP_Region& rn)
{
	//判断环的自交和方向
	int nl = rn.m_loopArray.size();
	for(int iL = 0; iL < nl; iL++)
	{
		if(!gb_checkLoopSelfIntersection(rn.m_loopArray[iL]))
			return false;
	}

	//判断环之间的边是否相交（主要是为了判断重合）
	for(int iL1 = 0; iL1 < nl - 1; iL1++)
	{
		for(int iL2 = iL1 + 1; iL2 < nl; iL2++)
		{
			if(gb_checkLoopIntersection(rn.m_loopArray[iL1], rn.m_loopArray[iL2]))
				return false;
		}
	}

	//判断内环之间是否有相互覆盖
	for(int iL1 = 1; iL1 < nl - 1; iL1++)
	{
		for(int iL2 = iL1 + 1; iL2 < nl; iL2++)
		{
			CP_BSPNode *a = gb_buildLoopBSPTree(rn.m_loopArray[iL1]);
			CP_BSPNode *b = gb_buildLoopBSPTree(rn.m_loopArray[iL2]);
			if(gb_tree1OverlapWithTree2(a, b))
				return false;
		}
	}

	//判断内环是否在外环内部
	for(int iL = 1; iL < nl; iL++)
	{
		CP_BSPNode *a = gb_buildLoopBSPTree(rn.m_loopArray[iL]);
		CP_BSPNode *b = gb_buildLoopBSPTree(rn.m_loopArray[0]);
		if(!gb_tree1InTree2(a, b))
			return false;
	}

	return true;
}

bool gb_checkLoopSelfIntersection(CP_Loop& ln)
{
	int np = ln.m_pointIDArray.size();
	bool clockwise = true;
	if(ln.m_loopIDinRegion == 0)
		clockwise = false;

	CP_Polygon* pn = ln.m_polygon;
	CP_Point *p0, *p1, *p2;
	int v1, v2;

	//判断是否为顺时针
	CP_Point *pc;

	int max;
	double xmax = -1000000;
	for(int iP = 0; iP < np; iP++){
		pc = &pn->m_pointArray[ln.m_pointIDArray[iP]];
		if(pc->m_x > xmax)
		{
			xmax = pc->m_x;
			max = iP;
		}
	}

	int min;
	double xmin = 1000000;
	for(int iP = max; iP < max + np; iP++){
		int iPm = iP % np;
		pc = &pn->m_pointArray[ln.m_pointIDArray[iPm]];
		if(pc->m_x < xmin)
		{
			xmin = pc->m_x;
			min = iPm;
		}
	}
	int minl = (min + np - 1) % np;
	int minr = (min + 1) % np;
	CP_Point &pminl = pn->m_pointArray[ln.m_pointIDArray[minl]];
	CP_Point &pmin = pn->m_pointArray[ln.m_pointIDArray[min]];
	CP_Point &pminr = pn->m_pointArray[ln.m_pointIDArray[minr]];
	double clock = (pmin.m_x - pminl.m_x) * (pminr.m_y - pmin.m_y) - (pmin.m_y - pminl.m_y) * (pminr.m_x - pmin.m_x);
	if(clock > 0 == clockwise){
		return false;
	}

	//判断xmin点所连线段是否与其他线段相交
	double a0, b0, c0, a, b, c;
	a0 = pmin.m_y - pminl.m_y;
	b0 = pminl.m_x - pmin.m_x;
	c0 = pmin.m_y * (pmin.m_x - pminl.m_x) - pmin.m_x * (pmin.m_y - pminl.m_y);
	for(int i = 0; i < np; i++){
		if(i == minl || i == min || i == (minl - 1 + np) % np)
			continue;
		CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
		//确定直线方程
		a = pl1.m_y - pl2.m_y;
		b = pl2.m_x - pl1.m_x;
		c = pl1.m_y * (pl1.m_x - pl2.m_x) - pl1.m_x * (pl1.m_y - pl2.m_y);

		double z1,z2,z3,z4;
		z1 = a0 * pl1.m_x + b0 * pl1.m_y + c0;
		z2 = a0 * pl2.m_x + b0 * pl2.m_y + c0;
		z3 = a * pmin.m_x + b * pmin.m_y + c;
		z4 = a * pminl.m_x + b * pminl.m_y + c;
		if(z1 * z2 < -TOLERENCE && z3 * z4 < -TOLERENCE)
			return false;
		if(z1 <= TOLERENCE && z1 >= -TOLERENCE){
			if((pl1.m_x - pmin.m_x) * (pl1.m_x - pminl.m_x) <= TOLERENCE)
				return false;
		}
		if(z2 <= TOLERENCE && z2 >= -TOLERENCE){
			if((pl2.m_x - pmin.m_x) * (pl2.m_x - pminl.m_x) <= TOLERENCE)
				return false;
		}
		if(z3 <= TOLERENCE && z3 >= -TOLERENCE){
			if((pl1.m_x - pmin.m_x) * (pl2.m_x - pmin.m_x) <= TOLERENCE)
				return false;
		}
		if(z4 <= TOLERENCE && z4 >= -TOLERENCE){
			if((pl1.m_x - pminl.m_x) * (pl2.m_x - pminl.m_x) <= TOLERENCE)
				return false;
		}

	}

	//确定直线方程
	a0 = pminr.m_y - pmin.m_y;
	b0 = pmin.m_x - pminr.m_x;
	c0 = pminr.m_y * (pminr.m_x - pmin.m_x) - pminr.m_x * (pminr.m_y - pmin.m_y);
	for(int i = 0; i < np; i++){
		if(i == minr || i == min || i == (min + np - 1) % np)
			continue;
		CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
		//确定直线方程
		a = pl1.m_y - pl2.m_y;
		b = pl2.m_x - pl1.m_x;
		c = pl1.m_y * (pl1.m_x - pl2.m_x) - pl1.m_x * (pl1.m_y - pl2.m_y);

		double z1,z2,z3,z4;
		z1 = a0 * pl1.m_x + b0 * pl1.m_y + c0;
		z2 = a0 * pl2.m_x + b0 * pl2.m_y + c0;
		z3 = a * pmin.m_x + b * pmin.m_y + c;
		z4 = a * pminr.m_x + b * pminr.m_y + c;
		if(z1 * z2 < -TOLERENCE && z3 * z4 < -TOLERENCE)
			return false;
		if(z1 <= TOLERENCE && z1 >= -TOLERENCE){
			if((pl1.m_x - pmin.m_x) * (pl1.m_x - pminr.m_x) <= TOLERENCE)
				return false;
		}
		if(z2 <= TOLERENCE && z2 >= -TOLERENCE){
			if((pl2.m_x - pmin.m_x) * (pl2.m_x - pminr.m_x) <= TOLERENCE)
				return false;
		}
		if(z3 <= TOLERENCE && z3 >= -TOLERENCE){
			if((pl1.m_x - pmin.m_x) * (pl2.m_x - pmin.m_x) <= TOLERENCE)
				return false;
		}
		if(z4 <= TOLERENCE && z4 >= -TOLERENCE){
			if((pl1.m_x - pminr.m_x) * (pl2.m_x - pminr.m_x) <= TOLERENCE)
				return false;
		}
	}

	//判断合法性
	vector<int> concavePoint;
	for(int iP = 0; iP < np; iP++)
	{
		v1 = iP - 1;
		if(iP == 0) v1 = np - 1;
		v2 = iP + 1;
		if(iP == np - 1) v2 = 0;

		p0 = &(pn->m_pointArray[ln.m_pointIDArray[v1]]);
		p1 = &(pn->m_pointArray[ln.m_pointIDArray[iP]]);
		p2 = &(pn->m_pointArray[ln.m_pointIDArray[v2]]);

		double cross = (p1->m_x - p0->m_x) * (p2->m_y - p1->m_y) - (p1->m_y - p0->m_y) * (p2->m_x - p1->m_x);
		if (cross == 0) continue;
		if((cross < 0) == !clockwise)
			concavePoint.push_back(iP);
	}

	for(unsigned int P = 0; P < concavePoint.size(); P++){
		int iP = concavePoint[P];
		p1 = &(pn->m_pointArray[ln.m_pointIDArray[iP]]);
		v1 = iP - 1;
		if(iP == 0) v1 = np - 1;
		p0 = &(pn->m_pointArray[ln.m_pointIDArray[v1]]);
		//确定直线方程
		
		a0 = p1->m_y - p0->m_y;
		b0 = p0->m_x - p1->m_x;
		c0 = p1->m_y * (p1->m_x - p0->m_x) - p1->m_x * (p1->m_y - p0->m_y);

		for(int i = 0; i < np; i++){
			if(i == v1 || i == iP || i == (v1 - 1 + np) % np)
				continue;
			CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
			//确定直线方程
			a = pl1.m_y - pl2.m_y;
			b = pl2.m_x - pl1.m_x;
			c = pl1.m_y * (pl1.m_x - pl2.m_x) - pl1.m_x * (pl1.m_y - pl2.m_y);

			double z1,z2,z3,z4;
			z1 = a0 * pl1.m_x + b0 * pl1.m_y + c0;
			z2 = a0 * pl2.m_x + b0 * pl2.m_y + c0;
			z3 = a * p1->m_x + b * p1->m_y + c;
			z4 = a * p0->m_x + b * p0->m_y + c;
			if(z1 * z2 < -TOLERENCE && z3 * z4 < -TOLERENCE)
				return false;
			if(a0 * pl1.m_x + b0 * pl1.m_y + c0 <= TOLERENCE && a0 * pl1.m_x + b0 * pl1.m_y + c0 >= -TOLERENCE){
				if((pl1.m_x - p1->m_x) * (pl1.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}
			if(a0 * pl2.m_x + b0 * pl2.m_y + c0 <= TOLERENCE && a0 * pl2.m_x + b0 * pl2.m_y + c0 >= -TOLERENCE){
				if((pl2.m_x - p1->m_x) * (pl2.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}
			if(a * p1->m_x + b * p1->m_y + c <= TOLERENCE && a * p1->m_x + b * p1->m_y + c >= -TOLERENCE){
				if((pl1.m_x - p1->m_x) * (pl2.m_x - p1->m_x) <= TOLERENCE)
					return false;
			}
			if(a * p0->m_x + b * p0->m_y + c <= TOLERENCE && a * p0->m_x + b * p0->m_y + c >= -TOLERENCE){
				if((pl1.m_x - p0->m_x) * (pl2.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}

		}

		v2 = iP + 1;
		if(iP == np - 1) v2 = 0;
		p0 = &(pn->m_pointArray[ln.m_pointIDArray[v2]]);
		//确定直线方程
		a0 = p1->m_y - p0->m_y;
		b0 = p0->m_x - p1->m_x;
		c0 = p1->m_y * (p1->m_x - p0->m_x) - p1->m_x * (p1->m_y - p0->m_y);
		for(int i = 0; i < np; i++){
			if(i == v2 || i == iP || i == (iP + np - 1) % np)
				continue;
			CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
			//确定直线方程
			a = pl1.m_y - pl2.m_y;
			b = pl2.m_x - pl1.m_x;
			c = pl1.m_y * (pl1.m_x - pl2.m_x) - pl1.m_x * (pl1.m_y - pl2.m_y);
	 
			if((a0 * pl1.m_x + b0 * pl1.m_y + c0) * (a0 * pl2.m_x + b0 * pl2.m_y + c0) < -TOLERENCE
				&& (a * p1->m_x + b * p1->m_y + c) * (a * p0->m_x + b * p0->m_y + c) < -TOLERENCE)
				return false;
			if(a0 * pl1.m_x + b0 * pl1.m_y + c0 <= TOLERENCE && a0 * pl1.m_x + b0 * pl1.m_y + c0 >= -TOLERENCE){
				if((pl1.m_x - p1->m_x) * (pl1.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}
			if(a0 * pl2.m_x + b0 * pl2.m_y + c0 <= TOLERENCE && a0 * pl2.m_x + b0 * pl2.m_y + c0 >= -TOLERENCE){
				if((pl2.m_x - p1->m_x) * (pl2.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}
			if(a * p1->m_x + b * p1->m_y + c <= TOLERENCE && a * p1->m_x + b * p1->m_y + c >= -TOLERENCE){
				if((pl1.m_x - p1->m_x) * (pl2.m_x - p1->m_x) <= TOLERENCE)
					return false;
			}
			if(a * p0->m_x + b * p0->m_y + c <= TOLERENCE && a * p0->m_x + b * p0->m_y + c >= -TOLERENCE){
				if((pl1.m_x - p0->m_x) * (pl2.m_x - p0->m_x) <= TOLERENCE)
					return false;
			}
		}
	}
	return true;
}

//存在相交的边则返回true，否则返回fasle
bool gb_checkRegionCrossRegion(CP_Region &region1, CP_Region &region2){
	vector<CP_Point> regionListBegin1, regionListBrgin2;
	vector<CP_Point> regionListEnd1, regionListEnd2;
	CP_Polygon *polygon = region1.m_polygon;
	for(unsigned int i = 0; i < region1.m_loopArray.size(); i++){
		CP_Loop &loop1 = region1.m_loopArray[i];
		for(unsigned int j = 0; j < loop1.m_pointIDArray.size(); j++){
			int k1 = (j + 1) % loop1.m_pointIDArray.size();

			for(unsigned int m = 0; m < region2.m_loopArray.size(); m++){
				CP_Loop &loop2 = region2.m_loopArray[m];
				for(unsigned int n = 0; n < loop2.m_pointIDArray.size(); n++){
					int k2 = (n + 1) % loop2.m_pointIDArray.size();
					CP_Point &p11 = polygon->m_pointArray[loop1.m_pointIDArray[j]];
					CP_Point &p12 = polygon->m_pointArray[loop1.m_pointIDArray[k1]];
					CP_Point &p21 = polygon->m_pointArray[loop2.m_pointIDArray[n]];
					CP_Point &p22 = polygon->m_pointArray[loop2.m_pointIDArray[k2]];
					if(gb_checkLineSegmentCross(&p11, &p12, &p21, &p22))
						return true;
				}
			}
		}
	}
	return false;
}

//存在相交的边则返回true，否则返回fasle
bool gb_checkLoopIntersection(CP_Loop& lnin1, CP_Loop& lnin2){
	int k1, k2;
	CP_Polygon *polygon = lnin1.m_polygon;
	CP_Point *p11, *p12, *p21, *p22;
	int np1 = lnin1.m_pointIDArray.size();
	int np2 = lnin2.m_pointIDArray.size();
	for(int i = 0; i < np1; i++){
		k1 = (i + 1) % np1;
		p11 = &polygon->m_pointArray[lnin1.m_pointIDArray[i]];
		p12 = &polygon->m_pointArray[lnin1.m_pointIDArray[k1]];
		for(int j = 0; j < np2; j++){
			k2 = (j + 1) % np2;
			p21 = &polygon->m_pointArray[lnin2.m_pointIDArray[j]];
			p22 = &polygon->m_pointArray[lnin2.m_pointIDArray[k2]];

			if(gb_checkLineSegmentCross(p11, p12, p21, p22))
				return true;
		}
	}
	return false;
}

//判断一个bsptree中是否有in的叶子节点
bool gb_treeHasInCell(CP_BSPNode* tree){
	if(gb_treeIsCell(tree)){
		if(tree->position == REGION_IN)
			return true;
		else 
			return false;
	}
	if(gb_treeHasInCell(tree->leftChild))
		return true;
	if(gb_treeHasInCell(tree->rightChild))
		return true;
	return false;
}

bool gb_tree1OverlapWithTree2(CP_BSPNode* tree1, CP_BSPNode* tree2){
	CP_BSPNode *result = gb_mergeBSPTree(tree1, tree2, CP_BSPOp::INTERSECTION);
	if(gb_treeHasInCell(result))
		return true;
	else
		return false;
}

bool gb_tree1InTree2(CP_BSPNode* tree1, CP_BSPNode* tree2){
	CP_BSPNode *result = gb_mergeBSPTree(tree1, tree2, CP_BSPOp::SUBTRACTION);
	if(gb_treeHasInCell(result))
		return false;
	else
		return true;
}

//相交就返回true，否则false
bool gb_checkLineSegmentCross(CP_Point* p11, CP_Point* p12, CP_Point* p21, CP_Point* p22){
	double ta, tb, tc, pa, pb, pc;
	CP_Point cross;
	//确定直线方程
	pa = p11->m_y - p12->m_y;
	pb = p12->m_x - p11->m_x;
	pc = p11->m_y * (p11->m_x - p12->m_x) - p11->m_x * (p11->m_y - p12->m_y);
	//确定直线方程
	ta = p21->m_y - p22->m_y;
	tb = p22->m_x - p21->m_x;
	tc = p21->m_y * (p21->m_x - p22->m_x) - p21->m_x * (p21->m_y - p22->m_y);

	if(ta * pb - tb * pa <= TOLERENCE && ta * pb - tb * pa >= -TOLERENCE){//平行
		if((ta * pc - tc * pa <= TOLERENCE && ta * pc - tc * pa >= -TOLERENCE) &&
			(tb * pc - tc * pb <= TOLERENCE && tb * pc - tc * pb >= -TOLERENCE)){ //直线重合
			if(pb * pb > pa * pa){ // x轴判断
				double min1 = p11->m_x, min2 = p21->m_x, max1 = p12->m_x, max2 = p22->m_x;
				if(min1 > max1){
					double c = min1;
					min1 = max1; 
					max1 = c;
				}
				if(min2 > max2){
					double c = min2;
					min2 = max2;
					max2 = c;
				}
				if(max2 <= min1 || max1 <= min2)
					return false;
				else
					return true;
			}
			else{ //y轴判断
				double min1 = p11->m_y, min2 = p21->m_y, max1 = p12->m_y, max2 = p22->m_y;
				if(min1 > max1){
					double c = min1;
					min1 = max1; 
					max1 = c;
				}
				if(min2 > max2){
					double c = min2;
					min2 = max2;
					max2 = c;
				}
				if(max2 <= min1 || max1 <= min2)
					return false;
				else
					return true;
			}
		}
	}
	else { //直线相交
		cross.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		cross.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		bool P1, P2;
		bool P1equal0 = false, P2equal0 = false;
		if(pb * pb > pa * pa){ // x轴判断
			if((p12->m_x - cross.m_x) * (cross.m_x - p11->m_x) >= -TOLERENCE){
				P1 = true;
				if((p12->m_x - cross.m_x) * (cross.m_x - p11->m_x) <= TOLERENCE)
					P1equal0 = true;
			}
			else
				P1 = false;
		}
		else{ //y轴判断
			if((p12->m_y - cross.m_y) * (cross.m_y - p11->m_y) >= -TOLERENCE){
				P1 = true;
				if((p12->m_y - cross.m_y) * (cross.m_y - p11->m_y) <= TOLERENCE)
					P1equal0 = true;
			}
			else
				P1 = false;
		}

		if(tb * tb > ta * ta){ // x轴判断
			if((p22->m_x - cross.m_x) * (cross.m_x - p21->m_x) >= -TOLERENCE){
				P2 = true;
				if((p22->m_x - cross.m_x) * (cross.m_x - p21->m_x) <= TOLERENCE)
					P2equal0 = true;
			}
			else
				P2 = false;
		}
		else{ //y轴判断
			if((p22->m_y - cross.m_y) * (cross.m_y - p21->m_y) >= -TOLERENCE){
				P2 = true;
				if((p22->m_y - cross.m_y) * (cross.m_y - p21->m_y) <= TOLERENCE)
					P2equal0 = true;
			}
			else
				P2 = false;
		}
		if(P1 && P2 && !(P1equal0 && P2equal0))
			return true;
	}
	return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////没有用到
//in存在点在region内部则return true，否则return false
bool gb_checkReginInRegin(CP_Region &in, CP_Region &region){
	vector<CP_Point> pointList;
	CP_Polygon *polygon = in.m_polygon;
	for(unsigned int i = 0; i < in.m_loopArray.size(); i++){
		CP_Loop loop = in.m_loopArray[i];
		for(unsigned int j = 0; j < loop.m_pointIDArray.size(); j++){
			pointList.push_back(polygon->m_pointArray[loop.m_pointIDArray[j]]);
		}
	}
	CP_BSPNode *regionTree = gb_buildRegionBSPTree(region);

	for(unsigned int i = 0; i < pointList.size(); i++){
		CP_Point point = pointList[i];
		CP_BSPNode *tree = regionTree;
		while(true){
			if(tree->leftChild == NULL && tree->rightChild == NULL){
				if(tree->position == REGION_IN){
					return true;
				}
				else return false;
			}
			CP_Partition *p = tree->partition;
			//判断point与p的位置关系
			double ax = p->end.m_x - p->begin.m_x;
			double ay = p->end.m_y - p->begin.m_y;
			double bx = point.m_x - p->end.m_x;
			double by = point.m_y - p->end.m_y;

			if(ax * by - bx * ay <= TOLERENCE && ax * by - bx * ay >= -TOLERENCE){ //点在直线上
				if(ax * ax > ay * ay){
					if((p->end.m_x - point.m_x) * (point.m_x - p->begin.m_x) >= 0){
						return true;
					}
				}
				else{
					if((p->end.m_y - point.m_y) * (point.m_y - p->begin.m_y) >= 0){
						return true;
					}
				}
			}
			else if(ax * by - bx * ay > TOLERENCE){
				tree = tree->leftChild;
			}
			else{
				tree = tree->rightChild;
			}
		}
	}

	return false;
}

// generate partition as 2d line segment(shoulnd't be 2d plane?)
void gb_getLoopPartition(CP_Loop& ln, vector<CP_Partition*>& vp){
	printf("\n\ngb_getLoopPartition\n");
	CP_Polygon *polygon = ln.m_polygon;
	int size = ln.m_pointIDArray.size();
	int direction = -1;
	if(ln.m_loopIDinRegion == 0)
		direction = 1;

	for(int i = 0; i < size; i++){
		int j = (i + direction + size) % size;
		printf("- (i,j) = (%d, %d)\n", i, j);
		CP_Partition *p = new CP_Partition();
		p->begin = polygon->m_pointArray[ln.m_pointIDArray[i]];
		p->end = polygon->m_pointArray[ln.m_pointIDArray[j]];
		vp.push_back(p);
		CP_PartitionList.push_back(p);
	}
}

CP_BSPNode* gb_buildPolygonBSPTree(CP_Polygon& pn){
	int nr = pn.m_regionArray.size();
	if(nr == 0){
		return NULL;
	}
	vector<CP_BSPNode *> bsptrees;
	CP_BSPNode* result = NULL;
	for(int iR = 0; iR < nr; iR++)
	{
		bsptrees.push_back(gb_buildRegionBSPTree(pn.m_regionArray[iR]));
	}
	if(nr == 1) return bsptrees[0];
	else{
		result = bsptrees[0];
		for(int iR = 1; iR < nr; iR++){
			result = gb_mergeBSPTree(result, bsptrees[iR], CP_BSPOp::UNION);
		}
	}
	return result;
}

CP_BSPNode* gb_buildRegionBSPTree(CP_Region& rn){
	printf("gb_buildRegionBSPTree\n");
	int nl = rn.m_loopArray.size();
	vector<CP_BSPNode *> bsptrees;
	CP_BSPNode* result;
	bsptrees.push_back(gb_buildLoopBSPTree(rn.m_loopArray[0]));

	if(nl == 1) return bsptrees[0];
	else{
		for(int iL = 1; iL < nl; iL++)
			bsptrees.push_back(gb_buildLoopBSPTree(rn.m_loopArray[iL]));

		result = bsptrees[0];
		for(int iL = 1; iL < nl; iL++)
			result = gb_mergeBSPTree(result, bsptrees[iL], CP_BSPOp::SUBTRACTION);

	}
	return result;
}

CP_BSPNode* gb_buildLoopBSPTree(CP_Loop& ln){
	printf("\tgb_buildLoopBSPTree\n");

	vector<CP_Partition*> partitionArray;
	gb_getLoopPartition(ln, partitionArray);

	CP_BSPNode *tree = NULL;
	tree = gb_buildBSPTree(partitionArray, NULL, CHILDINFO_NO); // root扼辑 CHILDINFO_NO甫 逞辫.
	return tree;
}

CP_BSPNode* gb_buildBSPTree(vector<CP_Partition*> &vp, CP_BSPNode* parent, char childInfo) {
	vector<CP_Partition*> F_right;
	vector<CP_Partition*> F_left;
	vector<CP_Partition*> F_coincident;

	CP_Partition *H = vp[0];
	CP_BSPNode *tree = new CP_BSPNode();
	CP_BSPNodeList.push_back(tree);
	tree->partition = vp[0];
	tree->parent = parent;
	if(childInfo == CHILDINFO_LEFT)
		parent->leftChild = tree;
	else if(childInfo == CHILDINFO_RIGHT)
		parent->rightChild = tree;

	// partitionLine is used to record the part of the line where the partition is located inside the area
	CP_Partition * partitionLine = new CP_Partition();
	CP_PartitionList.push_back(partitionLine);

	partitionLine->begin = tree->partition->begin;
	partitionLine->end = tree->partition->end;
	CP_Point pBegin, pEnd;
	double pmin, pmax, pcross;
	CP_Point point;
	if(!gb_p_in_region(tree, partitionLine, pBegin, pEnd, &point, pmin, pmax, pcross)){
		pBegin = tree->partition->end;
		pEnd = tree->partition->begin;
	}
	else{
		double vx = tree->partition->end.m_x - tree->partition->begin.m_x;
		double vy = tree->partition->end.m_y - tree->partition->begin.m_y;

		double l = sqrt(vx * vx + vy * vy);
	
		double mean_xy[2];
		mean_xy[0] = vx > 0 ? 1: -1;
		mean_xy[1] = vy > 0 ? 1: -1;

		int x_or_y = 0;

		if(vx * vx < vy * vy)
			x_or_y = 1;
		if(x_or_y == 0){
			pBegin.m_x = pmin * mean_xy[0] + tree->partition->begin.m_x;
			pEnd.m_x = pmax * mean_xy[0] + tree->partition->begin.m_x;
			pBegin.m_y = (pBegin.m_x - tree->partition->begin.m_x) * (vy / vx) + tree->partition->begin.m_y;
			pEnd.m_y = (pEnd.m_x - tree->partition->begin.m_x) * (vy / vx) + tree->partition->begin.m_y;
		}
		else{
			pBegin.m_y = pmin * mean_xy[1] + tree->partition->begin.m_y;
			pEnd.m_y = pmax * mean_xy[1] + tree->partition->begin.m_y;
			pBegin.m_x = (pBegin.m_y - tree->partition->begin.m_y) * (vx / vy) + tree->partition->begin.m_x;
			pEnd.m_x = (pEnd.m_y - tree->partition->begin.m_y) * (vx / vy) + tree->partition->begin.m_x;
		}
	}

	partitionLine->begin.m_x = pBegin.m_x;
	partitionLine->begin.m_y = pBegin.m_y;
	partitionLine->end.m_x = pEnd.m_x;
	partitionLine->end.m_y = pEnd.m_y;


	tree->pos_coincident.push_back(partitionLine);
	//partitionLine initialization ends

	if(vp.size() > 0)
		tree->pos_coincident.push_back(vp[0]);
	for(unsigned int i = 1; i < vp.size(); i++){
		char pos = getPatitionPos(vp, i, H);
		switch(pos){
		case POS_LEFT:
			F_left.push_back(vp[i]);
			break;
		case POS_POS_ON:
			tree->pos_coincident.push_back(vp[i]);
			break;
		case POS_NEG_ON:
			tree->neg_coincident.push_back(vp[i]);
			break;
		case POS_RIGHT:
			F_right.push_back(vp[i]);
			break;
		case POS_CROSS:
			CP_Partition *left = NULL;
			CP_Partition * right = NULL;
			gb_getCrossPartition(vp[i], H, left, right);
			F_left.push_back(left);
			F_right.push_back(right);
			break;
		}
	}

	if(F_left.size() == 0){
		tree->leftChild = new CP_BSPNode();
		CP_BSPNodeList.push_back(tree->leftChild);
		tree->leftChild->position = REGION_IN;
		tree->leftChild->parent = tree;
	}
	else {
		tree->leftChild = gb_buildBSPTree(F_left, tree, CHILDINFO_LEFT);
		tree->leftChild->parent = tree;
	}

	if(F_right.size() == 0){
		tree->rightChild = new CP_BSPNode();
		CP_BSPNodeList.push_back(tree->rightChild);
		tree->rightChild->position = REGION_OUT;
		tree->rightChild->parent = tree;
	}
	else {
		tree->rightChild = gb_buildBSPTree(F_right, tree, CHILDINFO_RIGHT);
		tree->rightChild->parent = tree;
	}
	return tree;
}

void gb_getCrossPartition(CP_Partition* T, CP_Partition* P, CP_Partition* &left, CP_Partition* &right){
	left = new CP_Partition();
	right = new CP_Partition();
	CP_PartitionList.push_back(left);

	CP_PartitionList.push_back(right);

	left->begin.m_x = T->begin.m_x;
	left->begin.m_y = T->begin.m_y;
	left->end.m_x = T->end.m_x;
	left->end.m_y = T->end.m_y;
	right->begin.m_x = T->begin.m_x;
	right->begin.m_y = T->begin.m_y;
	right->end.m_x = T->end.m_x;
	right->end.m_y = T->end.m_y;
	double pa, pb, pc, ta, tb, tc;
	ta =T->end.m_y - T->begin.m_y;
	tb =T->begin.m_x - T->end.m_x;
	tc = -ta * T->begin.m_x - tb * T->begin.m_y;

	pa =P->end.m_y - P->begin.m_y;
	pb =P->begin.m_x - P->end.m_x;
	pc = - pa * P->begin.m_x - pb * P->begin.m_y;

	CP_Point point;
	CP_PointList.push_back(point);
	point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
	point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

	if(-pb * ta + tb * pa > 0){
		left->begin = point;
		right->end = point;

	}
	else{
		left->end = point;
		right->begin = point;
	}
}

char getPatitionPos(vector<CP_Partition*> &vp, int pos, CP_Partition *H){
	double a, b, begin_a, begin_b, end_a, end_b;
	double begin_pos, end_pos;

	a = H->end.m_x - H->begin.m_x;
	b = H->end.m_y - H->begin.m_y;
	
	CP_Point vp_begin = vp[pos]->begin;
	CP_Point vp_end = vp[pos]->end;
	//判断起点

	begin_a = vp_begin.m_x - H->end.m_x;
	begin_b = vp_begin.m_y - H->end.m_y;

	begin_pos = a * begin_b - b * begin_a;

	//判断终点
	end_a = vp_end.m_x - H->end.m_x;
	end_b = vp_end.m_y - H->end.m_y;

	end_pos = a * end_b - b * end_a;

	//小于0，右边。等于0，on。大于0，左边
	if(end_pos <= TOLERENCE && end_pos >= -TOLERENCE)
		end_pos = 0;
	if(begin_pos <= TOLERENCE && begin_pos >= -TOLERENCE)
		begin_pos = 0;

	if(end_pos * begin_pos < 0){
		return POS_CROSS;
	}
	else if(end_pos * begin_pos > 0){
		if(end_pos < 0)
			return POS_RIGHT;
		else return POS_LEFT;
	}
	else{
		if(end_pos < 0 || begin_pos < 0)
			return POS_RIGHT;
		else if(end_pos > 0 || begin_pos > 0)
			return POS_LEFT;
		else {
			double a1, b1;
			a1 = vp_end.m_x - vp_begin.m_x;
			b1 = vp_end.m_y - vp_begin.m_y;
			if(a1 * a > 0 || b1 * b >0)
				return POS_POS_ON;
			else return POS_NEG_ON;
		}
	}
}

CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, CP_BSPNode* parent, CP_BSPOp op, bool left){

	CP_BSPNode* tree = NULL;
	CP_BSPNode* B_inRight = NULL;
	CP_BSPNode* B_inLeft = NULL;

	if(gb_treeIsCell(A) || gb_treeIsCell(B)){
		tree = gb_mergeTreeWithCell(A, B, op);
		tree->parent = parent;
		if(left) tree->parent->leftChild = tree;
		else tree->parent->rightChild = tree;
	}
	else{
		tree = new CP_BSPNode();
		CP_BSPNodeList.push_back(tree);
		tree->parent = parent;
		tree->partition = A->partition;

		for(auto &pc : A->pos_coincident)
			tree->pos_coincident.push_back(pc);

		for (auto& nc : A->neg_coincident)
			tree->neg_coincident.push_back(nc);

		CP_Point pBegin, pEnd;
		double pmin, pmax, pcross;
		CP_Point point;

		if(!gb_p_in_region(B, A->partition, pBegin, pEnd, &point, pmin, pmax, pcross)){
			pBegin = tree->partition->end;
			pEnd = tree->partition->begin;
		}
		else{
			double vx = tree->partition->end.m_x - tree->partition->begin.m_x;
			double vy = tree->partition->end.m_y - tree->partition->begin.m_y;
	
			double mean_xy[2];
			mean_xy[0] = vx > 0 ? 1: -1;
			mean_xy[1] = vy > 0 ? 1: -1;

			int x_or_y = 0;

			if(vx * vx < vy * vy)
				x_or_y = 1;
			if(x_or_y == 0){
				pBegin.m_x = pmin * mean_xy[0] + tree->partition->begin.m_x;
				pEnd.m_x = pmax * mean_xy[0] + tree->partition->begin.m_x;
				//pBegin.m_y = pmin * mean_xy[0] + tree->partition->begin.m_x;
				pBegin.m_y = (pBegin.m_x - tree->partition->begin.m_x) * (vy / vx) + tree->partition->begin.m_y;

				pEnd.m_y = (pEnd.m_x - tree->partition->begin.m_x) * (vy / vx) + tree->partition->begin.m_y;
			}
			else{
				pBegin.m_y = pmin * mean_xy[1] + tree->partition->begin.m_y;
				pEnd.m_y = pmax * mean_xy[1] + tree->partition->begin.m_y;
				//pBegin.m_y = pmin * mean_xy[0] + tree->partition->begin.m_x;
				pBegin.m_x = (pBegin.m_y - tree->partition->begin.m_y) * (vx / vy) + tree->partition->begin.m_x;
				pEnd.m_x = (pEnd.m_y - tree->partition->begin.m_y) * (vx / vy) + tree->partition->begin.m_x;
			}
		}
		gb_partitionBspt(B, tree->partition, B_inLeft, B_inRight, tree, pBegin, pEnd);

		if (left) tree->parent->leftChild = tree;
		else tree->parent->rightChild = tree;

		B_inLeft->parent = tree;
		B_inRight->parent = tree;
		tree->leftChild = B_inLeft;
		tree->rightChild = B_inRight;
		gb_mergeBSPTree(A->leftChild, B_inLeft, tree, op, true);
		gb_mergeBSPTree(A->rightChild, B_inRight, tree, op, false);		
	}

	return tree;
}

CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, CP_BSPOp op){

	CP_BSPNode* tree = NULL;
	CP_BSPNode* B_inRight = NULL;
	CP_BSPNode* B_inLeft = NULL;
	if(gb_treeIsCell(A) || gb_treeIsCell(B)){
		tree = gb_mergeTreeWithCell(A, B, op);
	}
	else{
		tree = new CP_BSPNode();
		CP_BSPNodeList.push_back(tree);

		tree->partition = A->partition;
		for(auto &pc : A->pos_coincident)
			tree->pos_coincident.push_back(pc);

		for(auto &nc : A->neg_coincident)
			tree->neg_coincident.push_back(nc);

		CP_Point pBegin, pEnd;
		double dx = tree->partition->end.m_x - tree->partition->begin.m_x;
		double dy = tree->partition->end.m_y - tree->partition->begin.m_y;
		double length = sqrt(dx * dx + dy * dy);
		dx = dx / length;
		dy = dy / length;
		pBegin.m_x = tree->partition->begin.m_x - dx * DBL_MAX / 2;
		pBegin.m_y = tree->partition->begin.m_y - dy * DBL_MAX / 2;
		pEnd.m_x = tree->partition->begin.m_x + dx * DBL_MAX / 2;
		pEnd.m_y = tree->partition->begin.m_y + dy * DBL_MAX / 2;

		gb_partitionBspt(B, tree->partition, B_inLeft, B_inRight, tree, pBegin, pEnd);
		B_inLeft->parent = tree;
		B_inRight->parent = tree;
		tree->leftChild = B_inLeft;
		tree->rightChild = B_inRight;
		gb_mergeBSPTree(A->leftChild, B_inLeft, tree, op, true);
		gb_mergeBSPTree(A->rightChild, B_inRight, tree, op, false);		
	}
	return tree;
}

CP_BSPNode* gb_mergeTreeWithCell(CP_BSPNode* T1, CP_BSPNode* T2, CP_BSPOp op){
	if(gb_treeIsCell(T1)){
		// Same as the Figure 5.1 in Naylor's paper.
		if(T1->position == REGION_IN){
			switch(op){
			case CP_BSPOp::UNION:
				return T1;
			case CP_BSPOp::INTERSECTION:
				return T2;
			case CP_BSPOp::SUBTRACTION:
				gb_complement(T2);
				return T2;
			}
		}
		else{
			switch(op){
			case CP_BSPOp::UNION:
				return T2;
			case CP_BSPOp::INTERSECTION:
				return T1;
			case CP_BSPOp::SUBTRACTION:
				return T1;
			}
		}
	}
	// Q : why below is different than naylor's work?!?!?!
	//if (gb_treeIsCell(T2)) return gb_mergeTreeWithCell(T2, T1, op); 
	else{
		if(T2->position == REGION_IN){
			switch(op){
			case CP_BSPOp::UNION:
				return T2;
			case CP_BSPOp::INTERSECTION:
				return T1;
			case CP_BSPOp::SUBTRACTION:
				CP_BSPNode *node = new CP_BSPNode();
				CP_BSPNodeList.push_back(node);
				node->position = REGION_OUT;
				return node;

				//(Q : why naylor's algorithm not working?
				//gb_complement(T1);
				//return T1;
			}
		}
		else{
			switch(op){
			case CP_BSPOp::UNION:
				return T1;
			case CP_BSPOp::INTERSECTION:
				return T2;
			case CP_BSPOp::SUBTRACTION:
				return T1;
				//return T2; (Q : why naylor's algorithm not working?
			}
		}
	}
	return NULL;
}

void gb_partitionBspt(CP_BSPNode* T, CP_Partition* partition, CP_BSPNode* & B_inLeft, CP_BSPNode*& B_inRight, CP_BSPNode* root, CP_Point& partitionBegin, CP_Point& partitionEnd){
	// if T is cell
	if(gb_treeIsCell(T)){
		B_inLeft = new CP_BSPNode(T);
		CP_BSPNodeList.push_back(B_inLeft);
		B_inRight = new CP_BSPNode(T);	
		CP_BSPNodeList.push_back(B_inRight);
		return;
	}

	// if T is not cell
	
	CP_Point cross_point;
	CP_Partition *partitionPush = NULL;
	CP_Partition *leftPartition = partition;
	CP_Partition *rightPartition = partition;
	CP_Point pLBegin, pLEnd, pRBegin, pREnd;
	pLBegin.m_x = partitionBegin.m_x;
	pLBegin.m_y = partitionBegin.m_y;
	pLEnd.m_x = partitionEnd.m_x;
	pLEnd.m_y = partitionEnd.m_y;
	pRBegin.m_x = partitionBegin.m_x;
	pRBegin.m_y = partitionBegin.m_y;
	pREnd.m_x = partitionEnd.m_x;
	pREnd.m_y = partitionEnd.m_y;

	// pos has 7 cases
	char pos = gb_t_p_Position3(T, partition, cross_point, pLBegin, pLEnd, pRBegin, pREnd);
	switch(pos){
	case P_T_ON_POS:
		B_inLeft = T->leftChild;
		B_inRight = T->rightChild;
		partitionPush = T->partition;
		root->pos_coincident.push_back(partitionPush);
		return;
	case P_T_ON_NEG:
		B_inLeft = T->rightChild;
		B_inRight = T->leftChild;
		partitionPush = T->partition;
		root->neg_coincident.push_back(partitionPush);
		return;
	case P_T_POS_NEG:
		B_inRight = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inRight);
		B_inRight->rightChild = T->rightChild;
		B_inRight->partition = T->partition;
		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			B_inRight->pos_coincident.push_back(T->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			B_inRight->neg_coincident.push_back(T->neg_coincident[i]);
		}
		gb_partitionBspt(T->leftChild, partition, B_inLeft, B_inRight->leftChild, root, pLBegin, pLEnd);
		break;
	case P_T_POS_POS:
		B_inLeft = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inLeft);

		B_inLeft->rightChild = T->rightChild;
		B_inLeft->partition = T->partition;
		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			B_inLeft->pos_coincident.push_back(T->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			B_inLeft->neg_coincident.push_back(T->neg_coincident[i]);
		}
		gb_partitionBspt(T->leftChild, partition, B_inLeft->leftChild, B_inRight, root, pLBegin, pLEnd);
		break;
	case P_T_NEG_POS:
		B_inLeft = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inLeft);

		B_inLeft->leftChild = T->leftChild;
		B_inLeft->partition = T->partition;
		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			B_inLeft->pos_coincident.push_back(T->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			B_inLeft->neg_coincident.push_back(T->neg_coincident[i]);
		}
		gb_partitionBspt(T->rightChild, partition, B_inLeft->rightChild, B_inRight, root, pRBegin, pREnd);
		break;
	case P_T_NEG_NEG:
		B_inRight = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inRight);
		B_inRight->leftChild = T->leftChild;
		B_inRight->partition = T->partition;
		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			B_inRight->pos_coincident.push_back(T->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			B_inRight->neg_coincident.push_back(T->neg_coincident[i]);
		}
		gb_partitionBspt(T->rightChild, partition, B_inLeft, B_inRight->rightChild, root, pRBegin, pREnd);
		break;
	case P_T_BOTH_POS:
		B_inLeft = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inLeft);
		B_inRight = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inRight);
		B_inRight->partition = T->partition;
		B_inLeft->partition = T->partition;

		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->pos_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();
				CP_PartitionList.push_back(left);
				CP_PartitionList.push_back(right);

				left->begin.m_x = T->pos_coincident[i]->begin.m_x;
				left->begin.m_y = T->pos_coincident[i]->begin.m_y;
				left->end.m_x = cross_point.m_x;
				left->end.m_y = cross_point.m_y;
				right->begin.m_x = cross_point.m_x;
				right->begin.m_y = cross_point.m_y;
				right->end.m_x = T->pos_coincident[i]->end.m_x;
				right->end.m_y = T->pos_coincident[i]->end.m_y;
				B_inLeft->pos_coincident.push_back(left);
				B_inRight->pos_coincident.push_back(right);
				break;
			case LINE_POS:
				B_inLeft->pos_coincident.push_back(T->pos_coincident[i]);
				break;
			case LINE_NEG:
				B_inRight->pos_coincident.push_back(T->pos_coincident[i]);
				break;
			}
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->neg_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();
				CP_PartitionList.push_back(left);
				CP_PartitionList.push_back(right);

				left->begin.m_x = cross_point.m_x;
				left->begin.m_y = cross_point.m_y;
				left->end.m_x = T->neg_coincident[i]->end.m_x;
				left->end.m_y = T->neg_coincident[i]->end.m_y;
				right->begin.m_x = T->neg_coincident[i]->begin.m_x;
				right->begin.m_y = T->neg_coincident[i]->begin.m_y;
				right->end.m_x = cross_point.m_x;
				right->end.m_y = cross_point.m_y;
				B_inLeft->neg_coincident.push_back(left);
				B_inRight->neg_coincident.push_back(right);
				break;
			case LINE_POS:
				B_inRight->neg_coincident.push_back(T->neg_coincident[i]);
				break;
			case LINE_NEG:
				B_inLeft->neg_coincident.push_back(T->neg_coincident[i]);
				break;
			}
		}
		gb_partitionBspt(T->leftChild, leftPartition, B_inLeft->leftChild, B_inRight->leftChild, root, pLBegin, pLEnd);
		gb_partitionBspt(T->rightChild, rightPartition, B_inLeft->rightChild, B_inRight->rightChild, root, pRBegin, pREnd);
		break;
	case P_T_BOTH_NEG:
		B_inLeft = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inLeft);
		B_inRight = new CP_BSPNode();
		CP_BSPNodeList.push_back(B_inRight);
		B_inRight->partition = T->partition;
		B_inLeft->partition = T->partition;

		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->pos_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();
				CP_PartitionList.push_back(left);

				CP_PartitionList.push_back(right);

				left->begin.m_x = cross_point.m_x;
				left->begin.m_y = cross_point.m_y;
				left->end.m_x = T->pos_coincident[i]->end.m_x;
				left->end.m_y = T->pos_coincident[i]->end.m_y;
				right->begin.m_x = T->pos_coincident[i]->begin.m_x;
				right->begin.m_y = T->pos_coincident[i]->begin.m_y;
				right->end.m_x = cross_point.m_x;
				right->end.m_y = cross_point.m_y;
				B_inLeft->pos_coincident.push_back(left);
				B_inRight->pos_coincident.push_back(right);
				break;
			case LINE_POS:
				B_inRight->pos_coincident.push_back(T->pos_coincident[i]);
				break;
			case LINE_NEG:
				B_inLeft->pos_coincident.push_back(T->pos_coincident[i]);
				break;
			}
		}
		for(unsigned int i = 0; i < T->neg_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->neg_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();
				CP_PartitionList.push_back(left);

				CP_PartitionList.push_back(right);

				left->begin.m_x = T->neg_coincident[i]->begin.m_x;
				left->begin.m_y = T->neg_coincident[i]->begin.m_y;
				left->end.m_x = cross_point.m_x;
				left->end.m_y = cross_point.m_y;
				right->begin.m_x = cross_point.m_x;
				right->begin.m_y = cross_point.m_y;
				right->end.m_x = T->neg_coincident[i]->end.m_x;
				right->end.m_y = T->neg_coincident[i]->end.m_y;

				B_inLeft->neg_coincident.push_back(left);
				B_inRight->neg_coincident.push_back(right);
				break;
			case LINE_POS:
				B_inLeft->neg_coincident.push_back(T->neg_coincident[i]);
				break;
			case LINE_NEG:
				B_inRight->neg_coincident.push_back(T->neg_coincident[i]);
				break;
			}
		}
		
		gb_partitionBspt(T->leftChild, leftPartition, B_inLeft->leftChild, B_inRight->leftChild, root, pLBegin, pLEnd);
		gb_partitionBspt(T->rightChild, rightPartition, B_inLeft->rightChild, B_inRight->rightChild, root, pRBegin, pREnd);
		break;
	}
	if(!gb_treeIsCell(B_inLeft)){
			B_inLeft->leftChild->parent = B_inLeft;
			B_inLeft->rightChild->parent = B_inLeft;
	}
	if(!gb_treeIsCell(B_inRight)){
			B_inRight->leftChild->parent = B_inRight;
			B_inRight->rightChild->parent = B_inRight;
	}
	 
}

char gb_coincidentPos(CP_Partition *p, CP_Point &point){
	CP_Point begin = p->begin;
	CP_Point end = p->end;

	if((point.m_x - begin.m_x > TOLERENCE && point.m_x - end.m_x < -TOLERENCE) 
		|| (point.m_x - begin.m_x < -TOLERENCE && point.m_x - end.m_x > TOLERENCE)
		|| (point.m_y - begin.m_y > TOLERENCE && point.m_y - end.m_y < -TOLERENCE)
		|| (point.m_y - begin.m_y < -TOLERENCE && point.m_y - end.m_y > TOLERENCE))
	{
		return LINE_IN;
	}
	else{
		double dx1, dy1, dx2, dy2;
		dx1 = point.m_x - begin.m_x;
		dy1 = point.m_y - begin.m_y;
		if(dx1 < 0) dx1 *= -1;
		if(dy1 < 0) dy1 *= -1;

		dx2 = point.m_x - end.m_x;
		dy2 = point.m_y - end.m_y;
		if(dx2 < 0) dx2 *= -1;
		if(dy2 < 0) dy2 *= -1;

		dx1 /= 2;
		dx2 /= 2;
		dy1 /= 2;
		dy2 /= 2;
		if(dx1 + dy1 < dx2 + dy2){
			return LINE_NEG;
		}
		else 
			return LINE_POS;
	}
}

char gb_t_p_Position(CP_BSPNode* A, CP_Partition* partition, CP_Point &cross_point, CP_Point& partitionLBegin, CP_Point& partitionLEnd, CP_Point& partitionRBegin, CP_Point& partitionREnd){
	CP_Partition *t_bp = A->partition;
	double pa, pb, pc, ta, tb, tc;
	ta =t_bp->end.m_y - t_bp->begin.m_y;
	tb =t_bp->begin.m_x - t_bp->end.m_x;
	tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

	pa =partition->end.m_y - partition->begin.m_y;
	pb =partition->begin.m_x - partition->end.m_x;
	pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

	if(ta * pb - tb * pa <= TOLERENCE && ta * pb - tb * pa >= -TOLERENCE){ // parallel
		if((ta * pc - tc * pa <= TOLERENCE && ta * pc - tc * pa >= -TOLERENCE) &&
			(tb * pc - tc * pb <= TOLERENCE && tb * pc - tc * pb >= -TOLERENCE)) // concide
		{
			if(ta * pa > 0 || tb * pb > 0){
				return P_T_ON_POS;
			}
			else {
				return P_T_ON_NEG;
			}
		}
		else{  //不相交
			double isleft = -tb * (partition->end.m_y - t_bp->end.m_y) -ta * (partition->end.m_x - t_bp->end.m_x);
			if(isleft > 0){ //P在T左边
				partitionRBegin.m_x = partition->end.m_x;
				partitionRBegin.m_y = partition->end.m_y;
				partitionREnd.m_x = partition->begin.m_x;
				partitionREnd.m_y = partition->begin.m_y;
				if(ta * pa > 0 || tb * pb > 0){
					return P_T_POS_NEG;
				}
				else{
					return P_T_POS_POS;
				}
			}
			else{//P在T右边
				partitionLBegin.m_x = partition->end.m_x;
				partitionLBegin.m_y = partition->end.m_y;
				partitionLEnd.m_x = partition->begin.m_x;
				partitionLEnd.m_y = partition->begin.m_y;
				if(ta * pa > 0 || tb * pb > 0){
					return P_T_NEG_POS;
				}
				else{
					return P_T_NEG_NEG;
				}
			}
		}
	}
	else{//相交
		//求t和p的交点
		CP_Point point;
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
		cross_point = CP_Point();
		CP_PointList.push_back(cross_point);
		cross_point.m_x = point.m_x;
		cross_point.m_y = point.m_y;

		//判断交点是否在partitionBegin和partitionEnd内部
		double dx = partitionLEnd.m_x - partitionLBegin.m_x;
		double dy = partitionLEnd.m_y - partitionLBegin.m_y;
		bool crossInpartition = false;
		if(pa * pa > pb * pb){ //y方向
			if(((partitionLEnd.m_y - point.m_y > TOLERENCE) && (point.m_y - partitionLBegin.m_y > TOLERENCE))
				|| ((partitionLEnd.m_y - point.m_y < -TOLERENCE) && (point.m_y - partitionLBegin.m_y < -TOLERENCE))){ //in
				crossInpartition = true;
			}
			if(partitionLEnd.m_y / pa - partitionLBegin.m_y / pa < 0)
				crossInpartition = false;
		}
		else{//x方向
			if(((partitionLEnd.m_x - point.m_x > TOLERENCE) && (point.m_x - partitionLBegin.m_x > TOLERENCE))
				|| ((partitionLEnd.m_x - point.m_x < -TOLERENCE) && (point.m_x - partitionLBegin.m_x < -TOLERENCE))){ //in
				crossInpartition = true;
			}
			if(partitionLEnd.m_x / (-pb) - partitionLBegin.m_x / (-pb) < 0)
				crossInpartition = false;
		}


		if(crossInpartition){
			if((-tb) * pa - (-pb) * ta > 0){
				partitionLBegin.m_x = point.m_x;
				partitionLBegin.m_y = point.m_y;
				partitionREnd.m_x = point.m_x;
				partitionREnd.m_y = point.m_y;
				return P_T_BOTH_POS;
			}
			else{
				partitionRBegin.m_x = point.m_x;
				partitionRBegin.m_y = point.m_y;
				partitionLEnd.m_x = point.m_x;
				partitionLEnd.m_y = point.m_y;
				return P_T_BOTH_NEG;
			}
		}
		else{		
			// 判断partition是否有可能继续包含在区域中的线段
			CP_Point begin, end;
			double pmin, pmax, pcross;
			CP_Point pos_point;
			double tmin, tmax, tcross;
			if(gb_p_in_region(A, partition, begin, end, &point, pmin, pmax, pcross)){

				
				CP_Partition *t_partition = new CP_Partition();
				CP_PartitionList.push_back(t_partition);

				t_partition->begin.m_x = A->partition->begin.m_x;
				t_partition->begin.m_y = A->partition->begin.m_y;
				t_partition->end.m_x = A->partition->end.m_x;
				t_partition->end.m_y = A->partition->end.m_y;

				gb_t_in_region(A, t_partition, pos_point, &point, tmin, tmax, tcross);


				CP_Partition *currentp = new CP_Partition();
				CP_PartitionList.push_back(currentp);

				currentp->begin.m_x = begin.m_x;
				currentp->begin.m_y = begin.m_y;
				currentp->end.m_x = end.m_x;
				currentp->end.m_y = end.m_y;

				double a = tmin - tcross;
				double b = tmax - tcross;

				if(a < 0) a *= -1;
				if(b < 0) b *= -1;

				double dirAP = a - b;

				a = pmin - pcross;
				b = pmax - pcross;

				if(a < 0) a *= -1;
				if(b < 0) b *= -1;

				double dirP = a - b;
				if(gb_t_p_left(pos_point, partition)){
					if(dirAP * dirP < 0){
						partitionRBegin.m_x = partition->end.m_x;
						partitionRBegin.m_y = partition->end.m_y;
						partitionREnd.m_x = partition->begin.m_x;
						partitionREnd.m_y = partition->begin.m_y;
						return P_T_POS_POS;
					}
					else{
						partitionLBegin.m_x = partition->end.m_x;
						partitionLBegin.m_y = partition->end.m_y;
						partitionLEnd.m_x = partition->begin.m_x;
						partitionLEnd.m_y = partition->begin.m_y;
						return P_T_NEG_POS;
					}
				}
				else{
					if(dirAP * dirP < 0){
						partitionLBegin.m_x = partition->end.m_x;
						partitionLBegin.m_y = partition->end.m_y;
						partitionLEnd.m_x = partition->begin.m_x;
						partitionLEnd.m_y = partition->begin.m_y;
						return P_T_NEG_NEG;
					}
					else{
						partitionRBegin.m_x = partition->end.m_x;
						partitionRBegin.m_y = partition->end.m_y;
						partitionREnd.m_x = partition->begin.m_x;
						partitionREnd.m_y = partition->begin.m_y;
						return P_T_POS_NEG;
					}
				}
			}
			else{
				partitionLBegin.m_x = partition->end.m_x;
				partitionLBegin.m_y = partition->end.m_y;
				partitionLEnd.m_x = partition->begin.m_x;
				partitionLEnd.m_y = partition->begin.m_y;
				partitionRBegin.m_x = partition->end.m_x;
				partitionRBegin.m_y = partition->end.m_y;
				partitionREnd.m_x = partition->begin.m_x;
				partitionREnd.m_y = partition->begin.m_y;
				if(gb_t_p_left(A->partition, partition)){
					return P_T_NEG_POS;
				}
				else{
					return P_T_NEG_NEG;
				}
			}
		}
	}

}

// classify 
char gb_t_p_Position3(CP_BSPNode* A, CP_Partition* partition, CP_Point &cross_point, 
	CP_Point& partitionLBegin, CP_Point& partitionLEnd, CP_Point& partitionRBegin, CP_Point& partitionREnd){

	CP_Partition *t_bp = A->partition;

	double pa, pb, pc, ta, tb, tc;
	ta =t_bp->end.m_y - t_bp->begin.m_y;
	tb =t_bp->begin.m_x - t_bp->end.m_x;
	tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

	pa =partition->end.m_y - partition->begin.m_y;
	pb =partition->begin.m_x - partition->end.m_x;
	pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

	bool not_in_region = false; // 
	if(pa * pa > pb * pb){ // y规氢
		if(partitionLEnd.m_y / pa - partitionLBegin.m_y / pa < 0)
			not_in_region = true;
	}
	else{ // x规氢
		if(partitionLEnd.m_x / (-pb) - partitionLBegin.m_x / (-pb) < 0)
			not_in_region = true;
	}

	if(not_in_region){
		if(gb_t_p_left(A->partition, partition)){
			return P_T_NEG_POS;
		}
		else{
			return P_T_NEG_NEG;
		}
	}
	else{
		if(ta * pb - tb * pa <= TOLERENCE && ta * pb - tb * pa >= -TOLERENCE){ // parallel
			if((ta * pc - tc * pa <= TOLERENCE && ta * pc - tc * pa >= -TOLERENCE) &&
				(tb * pc - tc * pb <= TOLERENCE && tb * pc - tc * pb >= -TOLERENCE)) // coincide
			{
				if(ta * pa > 0 || tb * pb > 0){
					return P_T_ON_POS;
				}
				else {
					return P_T_ON_NEG;
				}
			}
			else{  //不相交
				double isleft = -tb * (partition->end.m_y - t_bp->end.m_y) -ta * (partition->end.m_x - t_bp->end.m_x);
				if(isleft > 0){ //P在T左边
					partitionRBegin.m_x = partition->end.m_x;
					partitionRBegin.m_y = partition->end.m_y;
					partitionREnd.m_x = partition->begin.m_x;
					partitionREnd.m_y = partition->begin.m_y;
					if(ta * pa > 0 || tb * pb > 0){
						return P_T_POS_NEG;
					}
					else{
						return P_T_POS_POS;
					}
				}
				else{//P在T右边
					partitionLBegin.m_x = partition->end.m_x;
					partitionLBegin.m_y = partition->end.m_y;
					partitionLEnd.m_x = partition->begin.m_x;
					partitionLEnd.m_y = partition->begin.m_y;
					if(ta * pa > 0 || tb * pb > 0){
						return P_T_NEG_POS;
					}
					else{
						return P_T_NEG_NEG;
					}
				}
			}
		}
		else{//直线相交
			CP_Point point;
			point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
			point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
			cross_point = CP_Point();
			CP_PointList.push_back(cross_point);
			cross_point = point;
			bool crossInpartition = false;
			if(pa * pa > pb * pb){ //y方向
				if(((partitionLEnd.m_y - point.m_y > TOLERENCE) && (point.m_y - partitionLBegin.m_y > TOLERENCE))
					|| ((partitionLEnd.m_y - point.m_y < -TOLERENCE) && (point.m_y - partitionLBegin.m_y < -TOLERENCE))){ //in
					crossInpartition = true;
				}
			}
			else{//x方向
				if(((partitionLEnd.m_x - point.m_x > TOLERENCE) && (point.m_x - partitionLBegin.m_x > TOLERENCE))
					|| ((partitionLEnd.m_x - point.m_x < -TOLERENCE) && (point.m_x - partitionLBegin.m_x < -TOLERENCE))){ //in
					crossInpartition = true;
				}
			}

			if(crossInpartition){
				if((-tb) * pa - (-pb) * ta > 0){
					partitionLBegin.m_x = point.m_x;
					partitionLBegin.m_y = point.m_y;
					partitionREnd.m_x = point.m_x;
					partitionREnd.m_y = point.m_y;
					return P_T_BOTH_POS;
				}
				else{
					partitionRBegin.m_x = point.m_x;
					partitionRBegin.m_y = point.m_y;
					partitionLEnd.m_x = point.m_x;
					partitionLEnd.m_y = point.m_y;
					return P_T_BOTH_NEG;
				}
			}
			else{//区域内且不相交
				CP_Point begin, end;

				//计算p方向
				double a, b;
				if(pa * pa > pb * pb){ //y方向
					a = partitionLBegin.m_y - point.m_y;
					b = partitionLEnd.m_y - point.m_y;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				else{//x方向
					a = partitionLBegin.m_x - point.m_x;
					b = partitionLEnd.m_x - point.m_x;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				
				double dirP = a - b;
				if(dirP > 0)
					dirP = 1;
				else 
					dirP = -1;
				
				////计算t方向
				if(A->pos_coincident.size() == 0)
					int baa = 1;
				if(ta * ta > tb * tb){ //y方向
					a = A->pos_coincident[0]->begin.m_y - point.m_y;
					b = A->pos_coincident[0]->end.m_y - point.m_y;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				else{//x方向
					a = A->pos_coincident[0]->begin.m_x - point.m_x;
					b = A->pos_coincident[0]->end.m_x - point.m_x;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				
				double dirAP = a - b;
				if(dirAP > 0)
					dirAP = 1;
				else 
					dirAP = -1;

					
				if(gb_t_p_left(A->pos_coincident[0]->begin, partition)){
					if(dirAP * dirP < 0){
						partitionRBegin.m_x = partition->end.m_x;
						partitionRBegin.m_y = partition->end.m_y;
						partitionREnd.m_x = partition->begin.m_x;
						partitionREnd.m_y = partition->begin.m_y;
						return P_T_POS_POS;
					}
					else{
						partitionLBegin.m_x = partition->end.m_x;
						partitionLBegin.m_y = partition->end.m_y;
						partitionLEnd.m_x = partition->begin.m_x;
						partitionLEnd.m_y = partition->begin.m_y;
						return P_T_NEG_POS;
					}
				}
				else{
					if(dirAP * dirP < 0){
						partitionLBegin.m_x = partition->end.m_x;
						partitionLBegin.m_y = partition->end.m_y;
						partitionLEnd.m_x = partition->begin.m_x;
						partitionLEnd.m_y = partition->begin.m_y;
						return P_T_NEG_NEG;
					}
					else{
						partitionRBegin.m_x = partition->end.m_x;
						partitionRBegin.m_y = partition->end.m_y;
						partitionREnd.m_x = partition->begin.m_x;
						partitionREnd.m_y = partition->begin.m_y;
						return P_T_POS_NEG;
					}
				}

			}
		}
	}	
}

bool gb_point_partition_near_begin(CP_Partition* partition, CP_Point *point){
	double beginx, beginy, endx, endy;
	beginx = point->m_x - partition->begin.m_x;
	beginy = point->m_y - partition->begin.m_y;

	endx = point->m_x - partition->end.m_x;
	endy = point->m_y - partition->end.m_y;

	if(beginx < 0) beginx *= -1;
	if(beginy < 0) beginy *= -1;

	if(endx < 0) endx *= -1;
	if(endy < 0) endy *= -1;

	if(beginx + beginy < endx + endy)
		return true;
	else return false;
}

bool gb_t_p_left(CP_Partition* tp, CP_Partition* partition){
	double x1 = partition->end.m_x - partition->begin.m_x;
	double y1 = partition->end.m_y - partition->begin.m_y;

	double x2 = tp->end.m_x - partition->end.m_x;
	double y2 = tp->end.m_y - partition->end.m_y;

	if(x1 * y2 - x2 * y1 > 0)
		return true;
	else 
		return false;

}

bool gb_t_p_left(CP_Point &point, CP_Partition* partition){
	double x1 = partition->end.m_x - partition->begin.m_x;
	double y1 = partition->end.m_y - partition->begin.m_y;

	double x2 = point.m_x - partition->end.m_x;
	double y2 = point.m_y - partition->end.m_y;

	double lengthx1 = x1;
	double lengthy1 = y1;
	
	if(lengthx1 < 0)
		lengthx1 *= -1;
	if(lengthy1 < 0)
		lengthy1 *= -1;
	if(lengthx1 < lengthy1)
		lengthx1 = lengthy1;
	x1 /= lengthx1;
	y1 /= lengthx1;


	double lengthx2 = x2;
	double lengthy2 = y2;
	if(lengthx2 < 0)
		lengthx2 *= -1;
	if(lengthy2 < 0)
		lengthy2 *= -1;
	if(lengthx2 < lengthy2)
		lengthx2 = lengthy2;
	x2 /= lengthx2;
	y2 /= lengthx2;


	if(x1 * y2 - x2 * y1 > 0)
		return true;
	else 
		return false;

}

bool gb_p_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point& end, CP_Point *cross, double &pmin, double &pmax, double &pcross){
	begin.m_x = partition->begin.m_x;
	begin.m_y = partition->begin.m_y;
	end.m_x = partition->end.m_x;
	end.m_y = partition->end.m_y;

	double vx = partition->end.m_x - partition->begin.m_x;
	double vy = partition->end.m_y - partition->begin.m_y;
	
	double mean_xy[2];
	mean_xy[0] = vx > 0 ? 1: -1;
	mean_xy[1] = vy > 0 ? 1: -1;

	int x_or_y = 0;

	if(vx * vx < vy * vy)
		x_or_y = 1;
	

	double min = DBL_MAX / 2 * -1;
	double max = DBL_MAX / 2;

	CP_BSPNode *node = T;
	CP_BSPNode *child;
	double pa, pb, pc, ta, tb, tc;
	CP_Point point;
	while(node->parent != NULL){
		child = node;
		node = node->parent;
		CP_Partition* t_bp = new CP_Partition();
		CP_PartitionList.push_back(t_bp);

		if(child == node->leftChild){
			t_bp->begin.m_x = node->partition->begin.m_x;
			t_bp->begin.m_y = node->partition->begin.m_y;
			t_bp->end.m_x = node->partition->end.m_x;
			t_bp->end.m_y = node->partition->end.m_y;
		}
		else{
			t_bp->begin.m_x = node->partition->end.m_x;
			t_bp->begin.m_y = node->partition->end.m_y;
			t_bp->end.m_x = node->partition->begin.m_x;
			t_bp->end.m_y = node->partition->begin.m_y;
		}
		//CP_Partition* t_bp = node->partition;
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

		pa =partition->end.m_y - partition->begin.m_y;
		pb =partition->begin.m_x - partition->end.m_x;
		pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

		if((-tb) * pa - (-pb) * ta >= -TOLERENCE && (-tb) * pa - (-pb) * ta  <= TOLERENCE){//平行 现在假定重合或者平行在Tnode-partition左边都可以
			// node-partition向量（-tb,ta）
			double v1 = partition->begin.m_x - t_bp->begin.m_x;
			double v2 = partition->begin.m_y - t_bp->begin.m_y;
			if((-tb) * v2 - ta * v1 >= 0){
				continue;
			}
			else{
				return false;
			}
		}
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		if((-tb) * pa - (-pb) * ta > TOLERENCE)
		{
			if(x_or_y == 0){
				double currentMin = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						begin.m_x = point.m_x;
						begin.m_y = point.m_y;
					}
			}
			else if(x_or_y == 1){
				double currentMin = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						begin.m_x = point.m_x;
						begin.m_y = point.m_y;
					}
			}
		}
		else{
			if(x_or_y == 0){
				double currentMax = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						end.m_x = point.m_x;
						end.m_y = point.m_y;
					}
			}
			else if(x_or_y == 1){
				double currentMax = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						end.m_x = point.m_x;
						end.m_y = point.m_y;
					}
			}
		}
	}

	if(x_or_y == 0)
		pcross = (cross->m_x - partition->begin.m_x) * mean_xy[x_or_y];
	else if(x_or_y == 1)
		pcross = (cross->m_y - partition->begin.m_y) * mean_xy[x_or_y];
	pmin = min;
	pmax = max;
	return true;	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////没有使用，此函数用于将分割p修剪到只剩下属于T区域中的线段，有bug暂时没用
bool gb_cutPByRegionOfT(CP_BSPNode* T, CP_Partition* partition, CP_Point &pBegin, CP_Point& pEnd){
	CP_BSPNode* node = T;
	CP_BSPNode* child;
	CP_Point point;
	double pa, pb, pc, ta, tb, tc;
	pa =partition->end.m_y - partition->begin.m_y;
	pb =partition->begin.m_x - partition->end.m_x;
	pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

	double a1, b1, a2, b2;
	while(node->parent != NULL){
		child = node;
		node = node->parent;

		CP_Partition *t_bp = node->partition;
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;
		if(ta * pb - tb * pa <= TOLERENCE && ta * pb - tb * pa >= -TOLERENCE){}//平行
		else{
			point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
			point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
			bool crossInpartition = false;
			if(pa * pa > pb * pb){ //y方向
				if(((pEnd.m_y - point.m_y > TOLERENCE) && (point.m_y - pBegin.m_y > TOLERENCE))
					|| ((pEnd.m_y - point.m_y < -TOLERENCE) && (point.m_y - pBegin.m_y < -TOLERENCE))){ //in
					crossInpartition = true;
				}
			}
			else{//x方向
				if(((pEnd.m_x - point.m_x > TOLERENCE) && (point.m_x - pBegin.m_x > TOLERENCE))
					|| ((pEnd.m_x - point.m_x < -TOLERENCE) && (point.m_x - pBegin.m_x < -TOLERENCE))){ //in
					crossInpartition = true;
				}
			}
			if(crossInpartition){
				a1 = node->partition->end.m_x - node->partition->begin.m_x;
				b1 = node->partition->end.m_y - node->partition->begin.m_y;
				if(child == node->rightChild){
					a1 *= -1;
					b1 *= -1;
				}
				a2 = pBegin.m_x - node->partition->begin.m_x;
				b2 = pBegin.m_y - node->partition->begin.m_y;
				double a22 = a2, b22 = b2;
				if(a22 < 0)
					a22 = -a22;
				if(b22 < 0)
					b22 = -b22;
				if(b22 > a22)
					a22 = b22;
				a2 /= a22;
				b2 /= b22;

				if(a1 * b2 - a2 * b1 > TOLERENCE){
					pEnd.m_x = point.m_x;
					pEnd.m_y = point.m_y;
				}
				else{
					pBegin.m_x = point.m_x;
					pBegin.m_y = point.m_y;
				}
			}

		}
	}
	return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool gb_t_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &pos, CP_Point *cross, double &pmin, double &pmax, double &pcross){
	pos = partition->begin;
	double vx = partition->end.m_x - partition->begin.m_x;
	double vy = partition->end.m_y - partition->begin.m_y;

	double l = sqrt(vx * vx + vy * vy);
	
	double mean_xy[2];
	mean_xy[0] = vx > 0 ? 1: -1;
	mean_xy[1] = vy > 0 ? 1: -1;

	int x_or_y = 0;

	if(vx * vx < vy * vy)
		x_or_y = 1;
	

	double min = DBL_MAX * -1;
	double max = DBL_MAX;

	CP_BSPNode *node = T;
	CP_BSPNode *child;
	double pa, pb, pc, ta, tb, tc;
	CP_Point point;
	while(node->parent != NULL){
		child = node;
		node = node->parent;
		CP_Partition* t_bp = new CP_Partition();
		CP_PartitionList.push_back(t_bp);

		if(child == node->leftChild){
			t_bp->begin.m_x = node->partition->begin.m_x;
			t_bp->begin.m_y = node->partition->begin.m_y;
			t_bp->end.m_x = node->partition->end.m_x;
			t_bp->end.m_y = node->partition->end.m_y;
		}
		else{
			t_bp->begin.m_x = node->partition->end.m_x;
			t_bp->begin.m_y = node->partition->end.m_y;
			t_bp->end.m_x = node->partition->begin.m_x;
			t_bp->end.m_y = node->partition->begin.m_y;
		}
		//CP_Partition* t_bp = node->partition;
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

		pa =partition->end.m_y - partition->begin.m_y;
		pb =partition->begin.m_x - partition->end.m_x;
		pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

		if((-tb) * pa - (-pb) * ta >= -TOLERENCE && (-tb) * pa - (-pb) * ta  <= TOLERENCE){//平行 现在假定重合或者平行在Tnode-partition左边都可以
			// node-partition向量（-tb,ta）
			double v1 = partition->begin.m_x - t_bp->begin.m_x;
			double v2 = partition->begin.m_y - t_bp->begin.m_y;
			if((-tb) * v2 - ta * v1 >= 0){
				continue;
			}
			else{
				return false;
			}
		}
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		if((-tb) * pa - (-pb) * ta > TOLERENCE)
		{
			if(x_or_y == 0){
				double currentMin = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						pos = point;

					}
			}
			else if(x_or_y == 1){
				double currentMin = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						pos = point;
					}
			}
		}
		else{
			if(x_or_y == 0){
				double currentMax = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						pos = point;
					}
			}
			else if(x_or_y == 1){
				double currentMax = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						pos = point;
					}
			}
		}
	}

	if(x_or_y == 0)
		pcross = (cross->m_x - partition->begin.m_x) * mean_xy[x_or_y];
	else if(x_or_y == 1)
		pcross = (cross->m_y - partition->begin.m_y) * mean_xy[x_or_y];

	if(min >= -TOLERENCE && min <= TOLERENCE)
		min = 0;
	if(max >= -TOLERENCE && max <= TOLERENCE)
		max = 0;

	pmin = min;
	pmax = max;
	double posmax = max;
	double posmin = min;
	if(posmax < 0) posmax *= -1;
	if(posmin < 0) posmin *= -1;

	double mean;
	if(posmax + posmin == 0)
		mean = 0;
	else if(posmax == 0){
		mean = min / 2;
		if(mean > 1)
			mean = 1;
		else if(mean < -1)
			mean = -1;
	}
	else if(posmin == 0){
		mean = max / 2;
		if(mean > 1)
			mean = 1;
		else if(mean < -1)
			mean = -1;
	}
	else
	    mean = min * (posmax  / (posmax + posmin)) + max * (posmin / (posmax + posmin));
	if(x_or_y == 0){
		pos.m_x = partition->begin.m_x + mean * mean_xy[0];
		double disy = mean * vy * mean_xy[1]/ (vx * mean_xy[0]);
		pos.m_y = partition->begin.m_y + disy * mean_xy[1];
	}
	else if(x_or_y == 1){
		pos.m_y = partition->begin.m_y + mean * mean_xy[1];
		double disx = mean * vx * mean_xy[0]/ (vy * mean_xy[1]);
		pos.m_x = partition->begin.m_x + disx * mean_xy[0];
	}
	return true;	
}

bool gb_isCross(CP_BSPNode* A, CP_Point &point){ //需要考虑node是其parent的左孩子还是右孩子
	CP_BSPNode* node = A;
	CP_BSPNode* child;
	double a1, b1, a2, b2;
	while(node->parent != NULL){
		child = node;
		node = node->parent;
		a1 = node->partition->end.m_x - node->partition->begin.m_x;
		b1 = node->partition->end.m_y - node->partition->begin.m_y;
		if(child == node->rightChild){
			a1 *= -1;
			b1 *= -1;
		}

		a2 = point.m_x - node->partition->begin.m_x;
		b2 = point.m_y - node->partition->begin.m_y;
		if(a1 * b2 - b1 * a2 > TOLERENCE)
			continue;
		else
			return false;
	}
	return true;
}

bool gb_treeIsCell(CP_BSPNode* node){
	if(node->leftChild == NULL && node->rightChild == NULL)
		return true;
	else 
		return false;
}

bool gb_parent_t_sameDirection(CP_Partition *p1, CP_Partition *p2){
	double vxp1 = p1->end.m_x - p1->begin.m_x;
	double vyp1 = p1->end.m_y - p1->begin.m_y;

	double vxp2 = p2->end.m_x - p2->begin.m_x;
	double vyp2 = p2->end.m_y - p2->begin.m_y;

	double r = vxp1 * vyp2 - vxp2 * vyp1;
	if(r * r < TOLERENCE * TOLERENCE)
		r = 0;
	if(r > 0 || (r == 0 && p1->end.m_x == p2->begin.m_x && p1->end.m_y == p2->begin.m_y))
		return true;
	else
		return false;
}

void gb_complement(CP_BSPNode* T){
	if(gb_treeIsCell(T)){
		T->position = 3 - T->position;
		return;
	}
	gb_complement(T->leftChild);
	gb_complement(T->rightChild);

}

ofstream fout;
void debugBsptree(CP_BSPNode* T){	
	char filename[] = "debug.txt";
	fout.open(filename);
	debugFoutBsptree(T, 0);
	fout.close();
}

void debugFoutBsptree(CP_BSPNode* T, int floor){
	char *str = new char[floor + 1];
	for(int i = 0; i < floor; i++)
		str[i] = ' ';
	str[floor] = 0;
	if(T->position == 0){
		fout<<str<<"("<<T->partition->begin.m_x<<","<<T->partition->begin.m_y<<")---->("<<T->partition->end.m_x<<","<<T->partition->end.m_y<<")"<<endl;
		debugFoutBsptree(T->leftChild, floor + 3);
		debugFoutBsptree(T->rightChild, floor + 3);
	}
	else{	
		if(T->position == 1)
			fout<<str<<"IN"<<endl;
		else
			fout<<str<<"OUT"<<endl;
	}
	delete str;
}

void releaseMemory(){
	// erroneous
	/*
	for(unsigned int i = 0; i < CP_PointList.size(); i++){
		delete CP_PointList[i];
	}
	CP_PointList.clear();

	for(unsigned int i = 0; i < CP_PartitionList.size(); i++){
		delete CP_PartitionList[i];
	}
	CP_PartitionList.clear();

	for(unsigned int i = 0; i < CP_BSPNodeList.size(); i++){
		delete CP_BSPNodeList[i];
	}
	CP_BSPNodeList.clear();
	*/
}

bool gb_generateCellPolygon(CP_BSPNode *cell){
	CP_BSPNode *node = cell;
	CP_BSPNode *child = cell;
	while(node->parent != NULL){//generate polygon
		child = node;
		node = node->parent;
		CP_Partition *p = new CP_Partition();
		CP_PartitionList.push_back(p);

		for(unsigned int i = 1; i < node->pos_coincident.size(); i++){//因为0是记录的直线，用于判断T,P位置时记录T的partition在区域内的部分
			p->begin.m_x = node->pos_coincident[i]->begin.m_x;
			p->begin.m_y = node->pos_coincident[i]->begin.m_y;
			p->end.m_x = node->pos_coincident[i]->end.m_x;
			p->end.m_y = node->pos_coincident[i]->end.m_y;

			bool no_useful = false;
			for(unsigned int i = 0; i < cell->polygon.size(); i++){
				CP_Partition *face = cell->polygon[i];
				if(!gb_cutPolygonFace(p, face)){
					no_useful = true;
					break;
				}
			}

			if(!no_useful){//判断对node的polygon是否有贡献
				CP_Partition *node_face = new CP_Partition();
				CP_PartitionList.push_back(node_face);

				node_face->begin.m_x = p->begin.m_x;
				node_face->begin.m_y = p->begin.m_y;
				node_face->end.m_x = p->end.m_x;
				node_face->end.m_y = p->end.m_y;

				if(child == node->rightChild){
			
					if(cell->position == REGION_IN){
						node->rightIn.push_back(node_face);
					}
					else{
						node->rightOut.push_back(node_face);
					}

				}
				else{
					if(cell->position == REGION_IN){
						node->leftIn.push_back(node_face);
					}
					else{
						node->leftOut.push_back(node_face);
					}
				}
			}
		}

		for(unsigned int i = 0; i < node->neg_coincident.size(); i++){
			p->begin.m_x = node->neg_coincident[i]->begin.m_x;
			p->begin.m_y = node->neg_coincident[i]->begin.m_y;
			p->end.m_x = node->neg_coincident[i]->end.m_x;
			p->end.m_y = node->neg_coincident[i]->end.m_y;

			bool no_useful = false;
			for(unsigned int i = 0; i < cell->polygon.size(); i++){
				CP_Partition *face = cell->polygon[i];
				if(!gb_cutPolygonFace(p, face)){
					no_useful = true;
					break;
				}
			}

			if(!no_useful){//判断对node的polygon是否有贡献
				CP_Partition *node_face = new CP_Partition();
				CP_PartitionList.push_back(node_face);

				node_face->begin.m_x = p->begin.m_x;
				node_face->begin.m_y = p->begin.m_y;
				node_face->end.m_x = p->end.m_x;
				node_face->end.m_y = p->end.m_y;

				if(child == node->rightChild){
			
					if(cell->position == REGION_IN){
						node->rightIn.push_back(node_face);
					}
					else{
						node->rightOut.push_back(node_face);
					}

				}
				else{
					if(cell->position == REGION_IN){
						node->leftIn.push_back(node_face);
					}
					else{
						node->leftOut.push_back(node_face);
					}
				}
			}
		}
	}
	return true;
}

bool gb_generateCellPolygonPre(CP_BSPNode *cell){
	CP_BSPNode *node = cell;
	CP_BSPNode *child = cell;
	while(node->parent != NULL){//generate polygon
		child = node;
		node = node->parent;
		CP_Partition *p = new CP_Partition();
		CP_PartitionList.push_back(p);

		p->begin.m_x = node->partition->begin.m_x;
		p->begin.m_y = node->partition->begin.m_y;
		p->end.m_x = node->partition->end.m_x;
		p->end.m_y = node->partition->end.m_y;

		//判断对限制cell多边形的形状是否有贡献						
		CP_Partition *polygon_face = new CP_Partition();
		CP_PartitionList.push_back(polygon_face);

		polygon_face->begin.m_x = node->partition->begin.m_x;
		polygon_face->begin.m_y = node->partition->begin.m_y;
		polygon_face->end.m_x = node->partition->end.m_x;
		polygon_face->end.m_y = node->partition->end.m_y;
		CP_Point begin;
		CP_Point end;
		CP_PointList.push_back(begin);
		CP_PointList.push_back(end);
		if(gb_p_in_cellPolygon(cell, polygon_face, begin, end)){
			if(child == node->rightChild){
				gb_changePartitionDir(polygon_face);
			}
			cell->polygon.push_back(polygon_face);
		}

	}
	return true;
}

bool gb_generateCellPolygons(CP_BSPNode *node){
	if(gb_treeIsCell(node)){
		gb_generateCellPolygonPre(node);
		gb_generateCellPolygon(node);
		return true;
	}
	gb_generateCellPolygons(node->leftChild);
	gb_generateCellPolygons(node->rightChild);
	return true;
}

bool gb_changePartitionDir(CP_Partition *p){
	double x, y;
	x = p->begin.m_x;
	y = p->begin.m_y;

	p->begin.m_x = p->end.m_x;
	p->begin.m_y = p->end.m_y;

	p->end.m_x = x;
	p->end.m_y = y;
	return true;
}

bool gb_cutPolygonFace(CP_Partition *p, CP_Partition *face){
	double vx1, vx2, vx3, vy1, vy2, vy3;
	vx1 = face->end.m_x - face->begin.m_x;
	vy1 = face->end.m_y - face->begin.m_y;

	vx2 = p->begin.m_x - face->end.m_x;
	vy2 = p->begin.m_y - face->end.m_y;

	vx3 = p->end.m_x - face->end.m_x;
	vy3 = p->end.m_y - face->end.m_y;

	double begin = vx1 * vy2 - vy1 * vx2;
	double end = vx1 * vy3 - vy1 * vx3;

	if(begin * begin <= TOLERENCE * TOLERENCE)
		begin = 0;
	if(end * end <= TOLERENCE * TOLERENCE)
		end = 0;
	if(begin * end < 0){//cut
		double ta, tb, tc, pa, pb, pc;
		CP_Partition* t_bp = face;
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

		pa =p->end.m_y - p->begin.m_y;
		pb =p->begin.m_x - p->end.m_x;
		pc = - pa * p->begin.m_x - pb * p->begin.m_y;

		double x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		double y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		if(begin < 0){
			p->begin.m_x = x;
			p->begin.m_y = y;
		}
		else{
			p->end.m_x = x;
			p->end.m_y = y;
		}
		return true;
	}
	else if(begin + end > 0)
		return true;
	else if(begin + end < 0){
		return false;

	}
	else{ // p与face直线位置重合
		return true;
	}
}

bool gb_generateBSPTreeFaces(CP_BSPNode *node){
	if(gb_treeIsCell(node))
		return true;
	else{
		gb_generateBSPTreeFace(node);
	}

	gb_generateBSPTreeFaces(node->leftChild);
	gb_generateBSPTreeFaces(node->rightChild);
	return true;

}

bool gb_generateBSPTreeFace(CP_BSPNode *node){
	if(node->leftIn.size() * node->rightOut.size() != 0){
		for(unsigned int i = 0; i < node->leftIn.size(); i++){
			CP_Partition *p =  node->leftIn[i];
			CP_Partition *f;
			for(unsigned int j = 0; j < node->rightOut.size(); j++){
				f = node->rightOut[j];
				CP_Partition *result = new CP_Partition();
				CP_PartitionList.push_back(result);
			
				if(gb_cutParallelFace(p, f, result)){
					node->polygon.push_back(result);
				}
			}
		}
	}

	if(node->leftOut.size() * node->rightIn.size() != 0){
		for(unsigned int i = 0; i < node->leftOut.size(); i++){
			CP_Partition *p =  node->leftOut[i];
			CP_Partition *f;
			for(unsigned int j = 0; j < node->rightIn.size(); j++){
				f = node->rightIn[j];
				//取出f与p重合的部分
				CP_Partition *result = new CP_Partition();
				CP_PartitionList.push_back(result);
				if(gb_cutParallelFace(p, f, result)){
					node->polygon.push_back(result);
				}
			}
		}
	}
	return true;
}

bool gb_cutParallelFace(CP_Partition *p, CP_Partition *face, CP_Partition *result){
	int dx = 1, dy = 1;
	if(face->end.m_x - face->begin.m_x < 0)
		dx = -1;
	if(face->end.m_y - face->begin.m_y < 0)
		dy = -1;

	double face_begin = 0;
	double face_end = (face->end.m_x - face->begin.m_x) * dx + (face->end.m_y - face->begin.m_y) * dy;
	double p_begin = (p->begin.m_x - face->begin.m_x) * dx + (p->begin.m_y - face->begin.m_y) * dy;
	double p_end = (p->end.m_x - face->begin.m_x) * dx + (p->end.m_y - face->begin.m_y) * dy;

	if(p_end <= face_begin || p_begin >= face_end)
		return false;
	else if(p_begin <= face_begin && p_end >= face_end){
		result->begin.m_x = face->begin.m_x;
		result->begin.m_y = face->begin.m_y;
		result->end.m_x = face->end.m_x;
		result->end.m_y = face->end.m_y;
		return true;
	}
	else if(p_begin >= face_begin && p_end <= face_end){
		result->begin.m_x = p->begin.m_x;
		result->begin.m_y = p->begin.m_y;
		result->end.m_x = p->end.m_x;
		result->end.m_y = p->end.m_y;
		return true;
	}
	else if(p_begin <= face_begin && p_end > face_begin){
		result->begin.m_x = face->begin.m_x;
		result->begin.m_y = face->begin.m_y;
		result->end.m_x = p->end.m_x;
		result->end.m_y = p->end.m_y;
		return true;
	}
	else{
		result->begin.m_x = p->begin.m_x;
		result->begin.m_y = p->begin.m_y;
		result->end.m_x = face->end.m_x;
		result->end.m_y = face->end.m_y;
		return true;
	}
}

bool gb_p_in_cellPolygon(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point &end){
	begin.m_x = partition->begin.m_x;
	begin.m_y = partition->begin.m_y;
	end.m_x = partition->end.m_x;
	end.m_y = partition->end.m_y;

	double vx = partition->end.m_x - partition->begin.m_x;
	double vy = partition->end.m_y - partition->begin.m_y;

	double l = sqrt(vx * vx + vy * vy);
	
	double mean_xy[2];
	mean_xy[0] = vx > 0 ? 1: -1;
	mean_xy[1] = vy > 0 ? 1: -1;

	int x_or_y = 0;

	if(vx * vx < vy * vy)
		x_or_y = 1;
	

	double min = DBL_MAX * -1;
	double max = DBL_MAX;

	CP_BSPNode *node = T;
	double pa, pb, pc, ta, tb, tc;
	CP_Point point;
	for(unsigned int i = 0; i < node->polygon.size(); i++){
		CP_Partition* t_bp = node->polygon[i];
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

		pa =partition->end.m_y - partition->begin.m_y;
		pb =partition->begin.m_x - partition->end.m_x;
		pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

		if((-tb) * pa - (-pb) * ta >= -TOLERENCE && (-tb) * pa - (-pb) * ta <= TOLERENCE){//平行 现在假定重合或者平行在Tnode-partition左边都可以
			// node-partition向量（-tb,ta）
			double v1 = partition->begin.m_x - t_bp->begin.m_x;
			double v2 = partition->begin.m_y - t_bp->begin.m_y;
			if((-tb) * v2 - ta * v1 >= 0){
				continue;
			}
			else{
				return false;
			}
		}
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		if((-tb) * pa - (-pb) * ta > TOLERENCE)
		{
			if(x_or_y == 0){
				double currentMin = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						begin.m_x = point.m_x;
						begin.m_y = point.m_y;
					}
			}
			else if(x_or_y == 1){
				double currentMin = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						begin.m_x = point.m_x;
						begin.m_y = point.m_y;
					}
			}
		}
		else{
			if(x_or_y == 0){
				double currentMax = (point.m_x - partition->begin.m_x) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						end.m_x = point.m_x;
						end.m_y = point.m_y;
					}
			}
			else if(x_or_y == 1){
				double currentMax = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						end.m_x = point.m_x;
						end.m_y = point.m_y;
					}
			}
		}
	}

	return true;	
}