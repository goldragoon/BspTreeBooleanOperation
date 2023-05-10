// this project
#include "stdafx.h"
#include "CP_Polygon.h"

// std stuffs
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

void print_partition(const CP_Partition &_partition, const std::string _name = "partition") {
	printf("\t%s : (%.4lf, %.4lf) -> (%.4lf, %.4lf)\n", _name.c_str(),
		_partition.begin.m_x, _partition.begin.m_y,
		_partition.end.m_x, _partition.end.m_y);
}

bool equal_float(double x, double y, double epsilon) {
	if (fabs(x - y) <= epsilon)
		return true; //they are same
	return false; //they are not same
}

bool bigger_float(double x, double y, double epsilon) {
	if ((x - y) >= -epsilon)
		return true; //they are same
	return false; //they are not same
}

bool smaller_float(double x, double y, double epsilon) {
	if ((x - y) <= epsilon)
		return true; //they are same
	return false; //they are not same
}

void gb_distanceMinPointLoop(double&d, int& idRegion, int& idLoop,
                             CP_Point2& pt, CP_Polygon& pn)
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

void gb_distanceMinPointPolygon(double&d,int& id,CP_Point2& pt,CP_Polygon& pn)
{
    d = 0.0;
    id = -1;
    int n = pn.m_pointArray.size( );
    if (n<=0)
        return;
	d = pt.dist(pn.m_pointArray[0]);
    id = 0;
    int i;
    double dt;
    for (i=1; i<n; i++)
    {
		dt = pt.dist(pn.m_pointArray[i]);
        if (dt < d)
        {
            d = dt;
            id = i;
        }
    }
}

double gb_distancePointSegment(CP_Point2& pt, CP_Point2& p1, CP_Point2& p2)
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
        d = d01*d01/d0;
        d = d1 - d;
        d = sqrt(d);
        return d;
    }
    if (d1>d2)
        d = d2;
    else d = d1;
    d = sqrt(d);
    return d;
}

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
                }
            }
        }
    }
    return false;
}

void gb_insertPointInPolygon(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, CP_Point2& newPoint)
{
    int nv = pn.m_pointArray.size( );
    pn.m_pointArray.push_back(newPoint);
    pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.insert(
        pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.begin( )+idPointInLoop+1,
        nv);
}

void gb_intArrayInit(VT_IntArray& vi, int data)
{
    int n = vi.size( );
    int i;
    for (i=0; i<n; i++)
        vi[i] = data;
}

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
    }
    gb_intArrayInitPointSame(vi, pn, eT);
}

void gb_intArrayInitPoint(VT_IntArray& vi, CP_Polygon& pn, int v, double eT)
{
    int n = pn.m_pointArray.size( );
    if (n<=0)
    {
        vi.clear( );
        return;
    }
    vi.resize(n);
    int i;
    double d;
 for (i=0; i<n; i++)
    {
        if (i==v)
            vi[i] = i;
        else
        {
			d = pn.m_pointArray[i].dist(pn.m_pointArray[v]);
            if (d <= eT) vi[i] = i;
            else vi[i] = -1;
        }
    }
}

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
					d = pn.m_pointArray[i].dist(pn.m_pointArray[j]);
                    if (d<=eT)
                        vi[j] = j;
                }
            }
        }
    }
}

void gb_intArrayInitPolygon(VT_IntArray& vi, CP_Polygon& pn)
{
    int i;
    int n = pn.m_pointArray.size( );
    vi.resize(n);
    for (i=0; i<n; i++)
        vi[i] = i;
}

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
    } // if써監
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
				da = ps.m_pointArray[i].dist(pr.m_pointArray[j]);
                if (da <= eT)
                    vr[j] = j;
            }
        }
    }
}

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
        }
    }
    gb_intArrayInitPointSame(vi, pn, eT);
}

void gb_moveLoop(CP_Polygon& pn, int idRegion, int idLoop, double vx, double vy)
{
	int nr, nL, nv;
	int i, id;
	nr = pn.m_regionArray.size();
	if ((idRegion < 0) || (idRegion >= nr))
		return;
	nL = pn.m_regionArray[idRegion].m_loopArray.size();
	if ((idLoop < 0) || (idLoop >= nL))
		return;
	nv = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray.size();
	for (i = 0; i < nv; i++)
	{
		id = pn.m_regionArray[idRegion].m_loopArray[idLoop].m_pointIDArray[i];
		pn.m_pointArray[id].m_x += vx;
		pn.m_pointArray[id].m_y += vy;
	}
}

void gb_movePoint(CP_Polygon& pn, int id, double vx, double vy)
{
    int n = pn.m_pointArray.size( );
    if ((id<0) || (id>=n))
        return;
    pn.m_pointArray[id].m_x += vx;
    pn.m_pointArray[id].m_y += vy;
}

void gb_movePointIntArray(CP_Polygon& pn, VT_IntArray& vi, double vx, double vy)
{
    int n = vi.size( );
    int i;
    for (i=0; i<n; i++)
        gb_movePoint(pn, vi[i], vx, vy);
}

void gb_movePolygon(CP_Polygon& pn, double vx, double vy)
{
	int n = pn.m_pointArray.size();
	int i;
	for (i = 0; i < n; i++)
	{
		pn.m_pointArray[i].m_x += vx;
		pn.m_pointArray[i].m_y += vy;
	}
}

void gb_moveRegion(CP_Polygon& pn, int idRegion, double vx, double vy)
{
	int nr, nL, nv;
	int i, j, k, id;
	nr = pn.m_regionArray.size();
	if ((idRegion < 0) || (idRegion >= nr))
		return;
	i = idRegion;
	nL = pn.m_regionArray[i].m_loopArray.size();
	for (j = 0; j < nL; j++)
	{
		nv = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray.size();
		for (k = 0; k < nv; k++)
		{
			id = pn.m_regionArray[i].m_loopArray[j].m_pointIDArray[k];
			pn.m_pointArray[id].m_x += vx;
			pn.m_pointArray[id].m_y += vy;
		}
	}
}

// 쉥瞳홍애麟깃溝苟돨듐瘻뻣냥槨瞳팁캥麟깃苟돨듐
// result:      渴놔돨瞳팁캥麟깃苟돨듐;
// pointGlobal: 渴흙돨瞳홍애麟깃溝苟돨듐;
// scale:       渴흙돨궐절凜綾;
// translation: 渴흙돨틱盧麟깃令。
void gb_pointConvertFromGlobalToScreen(CP_Point2& result, CP_Point2 pointGlobal, double scale, CP_Point2 translation, int screenX, int screenY)
{
    result.m_x=(pointGlobal.m_x-translation.m_x)*scale;
    result.m_y=(pointGlobal.m_y-translation.m_y)*scale;
    result.m_x+= (screenX/2);
    result.m_y=screenY/2-result.m_y;
} // 변鑒PointConvertFromGlobalToScreen써監

// 쉥瞳팁캥麟깃苟돨듐瘻뻣냥槨瞳홍애麟깃溝苟돨듐
// result:      渴놔돨瞳홍애麟깃溝苟돨듐;
// pointScreen: 渴흙돨瞳팁캥麟깃溝苟돨듐;
// scale:       渴흙돨궐절凜綾;
// translation: 渴흙돨틱盧麟깃令。
void gb_pointConvertFromScreenToGlobal(CP_Point2& result, CP_Point2 pointScreen, double scale, CP_Point2 translation, int screenX, int screenY)
{
    result.m_x=pointScreen.m_x - screenX/2;
    result.m_y=screenY/2-pointScreen.m_y;
    result.m_x=result.m_x/scale+translation.m_x;
    result.m_y=result.m_y/scale+translation.m_y;
}

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
    }
    p.m_regionArray[idRegion].m_loopArray[nL].m_polygon = &p;
    p.m_regionArray[idRegion].m_loopArray[nL].m_regionIDinPolygon = idRegion;
    p.m_regionArray[idRegion].m_loopArray[nL].m_loopIDinRegion = nL;
    p.m_regionArray[idRegion].m_loopArray[nL].m_pointIDArray.resize(n);
    for (i=0, k= t-1; i<n; i++, k--)
    {
        p.m_regionArray[idRegion].m_loopArray[nL].m_pointIDArray[i] = k;
    }
    return true;
}

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
    } // for써監
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
    } // for써監
} // 변鑒gb_polygonNewOutLoopRegular써監

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
    }
    pn.m_regionArray[idRegion].m_loopArray.erase(
        pn.m_regionArray[idRegion].m_loopArray.begin( )+idLoop);
    return true;
}
bool gb_removePoint(CP_Polygon& pn, int id)
{
    int ir, iL, iLv, nLv;
    bool rf = gb_findPointInLoop(pn, ir, iL, iLv, id);
    if (!rf)
        return false;
    nLv = pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.size( );
    if (nLv<4)
        return (gb_removeLoop(pn, ir, iL));
    pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.erase(
        pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.begin( )+iLv);
    pn.m_pointArray.erase(pn.m_pointArray.begin( )+id);
    gb_subtractOneAboveID(pn, id);
    return true;
}
bool gb_removeRegion(CP_Polygon& pn, int idRegion)
{
    int nr, nL, nLv, iL, iLv, v;
    nr = pn.m_regionArray.size( );
    if (nr<2)
    {
        pn.mb_clear( );
        return true;
    } // if써監`
    nL = pn.m_regionArray[idRegion].m_loopArray.size( );
    for (iL=0; iL<nL; iL++)
    {
        nLv =  pn.m_regionArray[idRegion].m_loopArray[iL].m_pointIDArray.size( );
        for (iLv=0; iLv<nLv; iLv++)
        {
            v = pn.m_regionArray[idRegion].m_loopArray[iL].m_pointIDArray[iLv];
            pn.m_pointArray.erase(pn.m_pointArray.begin( )+v);
            gb_subtractOneAboveID(pn, v);
        }
    }
    pn.m_regionArray.erase(pn.m_regionArray.begin( )+idRegion);
    return true;
}

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

bool gb_checkPolygon(CP_Polygon& pn)
{
	//쇱꿴데몸혐堵
	int nr = pn.m_regionArray.size();
	for(int iR = 0; iR < nr; iR++)
	{
		if(!gb_checkRegion(pn.m_regionArray[iR]))
		{
			return false;
		}
	}

	//긋宮슥
	for(int i = 0; i < nr - 1; i++){
		for(int j = i + 1; j < nr; j++){
			if(gb_checkRegionCrossRegion(pn.m_regionArray[i], pn.m_regionArray[j]))
				return false;
		}
	}

	//쇱꿴혐堵裂쇌角뤠唐혐堵路북
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
	//털뙤뻔돨菱슥뵨렘蕨
	int nl = rn.m_loopArray.size();
	for(int iL = 0; iL < nl; iL++)
	{
		if(!gb_checkLoopSelfIntersection(rn.m_loopArray[iL]))
			return false;
	}

	//털뙤뻔裂쇌돨긋角뤠宮슥（寮狼角槨죄털뙤路북）
	for(int iL1 = 0; iL1 < nl - 1; iL1++)
	{
		for(int iL2 = iL1 + 1; iL2 < nl; iL2++)
		{
			if(gb_checkLoopIntersection(rn.m_loopArray[iL1], rn.m_loopArray[iL2]))
				return false;
		}
	}

	//털뙤코뻔裂쇌角뤠唐宮빳림맨
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

	//털뙤코뻔角뤠瞳棍뻔코꼬
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
	CP_Point2 *p0, *p1, *p2;
	int v1, v2;

	//털뙤角뤠槨糠珂濾
	CP_Point2 *pc;

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
	CP_Point2 &pminl = pn->m_pointArray[ln.m_pointIDArray[minl]];
	CP_Point2 &pmin = pn->m_pointArray[ln.m_pointIDArray[min]];
	CP_Point2 &pminr = pn->m_pointArray[ln.m_pointIDArray[minr]];
	double clock = (pmin.m_x - pminl.m_x) * (pminr.m_y - pmin.m_y) - (pmin.m_y - pminl.m_y) * (pminr.m_x - pmin.m_x);
	if(clock > 0 == clockwise){
		return false;
	}

	//털뙤xmin듐杰젯窟뙈角뤠宅페儉窟뙈宮슥
	double a0, b0, c0, a, b, c;
	a0 = pmin.m_y - pminl.m_y;
	b0 = pminl.m_x - pmin.m_x;
	c0 = pmin.m_y * (pmin.m_x - pminl.m_x) - pmin.m_x * (pmin.m_y - pminl.m_y);
	for(int i = 0; i < np; i++){
		if(i == minl || i == min || i == (minl - 1 + np) % np)
			continue;
		CP_Point2 &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point2 &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
		//횅땍殮窟렘넋
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

	//횅땍殮窟렘넋
	a0 = pminr.m_y - pmin.m_y;
	b0 = pmin.m_x - pminr.m_x;
	c0 = pminr.m_y * (pminr.m_x - pmin.m_x) - pminr.m_x * (pminr.m_y - pmin.m_y);
	for(int i = 0; i < np; i++){
		if(i == minr || i == min || i == (min + np - 1) % np)
			continue;
		CP_Point2 &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point2 &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
		//횅땍殮窟렘넋
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

	//털뙤북랬昑
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
		//횅땍殮窟렘넋
		
		a0 = p1->m_y - p0->m_y;
		b0 = p0->m_x - p1->m_x;
		c0 = p1->m_y * (p1->m_x - p0->m_x) - p1->m_x * (p1->m_y - p0->m_y);

		for(int i = 0; i < np; i++){
			if(i == v1 || i == iP || i == (v1 - 1 + np) % np)
				continue;
			CP_Point2 &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point2 &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
			//횅땍殮窟렘넋
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
		//횅땍殮窟렘넋
		a0 = p1->m_y - p0->m_y;
		b0 = p0->m_x - p1->m_x;
		c0 = p1->m_y * (p1->m_x - p0->m_x) - p1->m_x * (p1->m_y - p0->m_y);
		for(int i = 0; i < np; i++){
			if(i == v2 || i == iP || i == (iP + np - 1) % np)
				continue;
			CP_Point2 &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point2 &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
			//횅땍殮窟렘넋
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

//닸瞳宮슥돨긋橙럿쀼true，뤠橙럿쀼fasle
bool gb_checkRegionCrossRegion(CP_Region &region1, CP_Region &region2){
	vector<CP_Point2> regionListBegin1, regionListBrgin2;
	vector<CP_Point2> regionListEnd1, regionListEnd2;
	CP_Polygon *polygon = region1.m_polygon;
	for(unsigned int i = 0; i < region1.m_loopArray.size(); i++){
		CP_Loop &loop1 = region1.m_loopArray[i];
		for(unsigned int j = 0; j < loop1.m_pointIDArray.size(); j++){
			int k1 = (j + 1) % loop1.m_pointIDArray.size();

			for(unsigned int m = 0; m < region2.m_loopArray.size(); m++){
				CP_Loop &loop2 = region2.m_loopArray[m];
				for(unsigned int n = 0; n < loop2.m_pointIDArray.size(); n++){
					int k2 = (n + 1) % loop2.m_pointIDArray.size();
					CP_Point2 &p11 = polygon->m_pointArray[loop1.m_pointIDArray[j]];
					CP_Point2 &p12 = polygon->m_pointArray[loop1.m_pointIDArray[k1]];
					CP_Point2 &p21 = polygon->m_pointArray[loop2.m_pointIDArray[n]];
					CP_Point2 &p22 = polygon->m_pointArray[loop2.m_pointIDArray[k2]];
					if(gb_checkLineSegmentCross(&p11, &p12, &p21, &p22))
						return true;
				}
			}
		}
	}
	return false;
}

//닸瞳宮슥돨긋橙럿쀼true，뤠橙럿쀼fasle
bool gb_checkLoopIntersection(CP_Loop& lnin1, CP_Loop& lnin2){
	int k1, k2;
	CP_Polygon *polygon = lnin1.m_polygon;
	CP_Point2 *p11, *p12, *p21, *p22;
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

bool gb_treeHasInCell(CP_BSPNode* tree){
	if(tree->isCell()){
		if(tree->side == CP_BSPNode::Sideness::INSIDE)
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
	CP_BSPNode *result = gb_mergeBSPTree_root(tree1, tree2, CP_BSPOp::INTERSECTION);
	if(gb_treeHasInCell(result))
		return true;
	else
		return false;
}

bool gb_tree1InTree2(CP_BSPNode* tree1, CP_BSPNode* tree2){
	CP_BSPNode *result = gb_mergeBSPTree_root(tree1, tree2, CP_BSPOp::SUBTRACTION);
	if(gb_treeHasInCell(result))
		return false;
	else
		return true;
}

bool gb_checkLineSegmentCross(CP_Point2* p11, CP_Point2* p12, CP_Point2* p21, CP_Point2* p22){
	double ta, tb, tc, pa, pb, pc;
	CP_Point2 cross;
	pa = p11->m_y - p12->m_y;
	pb = p12->m_x - p11->m_x;
	pc = p11->m_y * (p11->m_x - p12->m_x) - p11->m_x * (p11->m_y - p12->m_y);

	ta = p21->m_y - p22->m_y;
	tb = p22->m_x - p21->m_x;
	tc = p21->m_y * (p21->m_x - p22->m_x) - p21->m_x * (p21->m_y - p22->m_y);

	if(ta * pb - tb * pa <= TOLERENCE && ta * pb - tb * pa >= -TOLERENCE){//틱契
		if((ta * pc - tc * pa <= TOLERENCE && ta * pc - tc * pa >= -TOLERENCE) &&
			(tb * pc - tc * pb <= TOLERENCE && tb * pc - tc * pb >= -TOLERENCE)){ //殮窟路북
			if(pb * pb > pa * pa){ // x菉털뙤
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
			else{ //y菉털뙤
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
	else { //殮窟宮슥
		cross.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		cross.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);

		bool P1, P2;
		bool P1equal0 = false, P2equal0 = false;
		if(pb * pb > pa * pa){ // x菉털뙤
			if((p12->m_x - cross.m_x) * (cross.m_x - p11->m_x) >= -TOLERENCE){
				P1 = true;
				if((p12->m_x - cross.m_x) * (cross.m_x - p11->m_x) <= TOLERENCE)
					P1equal0 = true;
			}
			else
				P1 = false;
		}
		else{ //y菉털뙤
			if((p12->m_y - cross.m_y) * (cross.m_y - p11->m_y) >= -TOLERENCE){
				P1 = true;
				if((p12->m_y - cross.m_y) * (cross.m_y - p11->m_y) <= TOLERENCE)
					P1equal0 = true;
			}
			else
				P1 = false;
		}

		if(tb * tb > ta * ta){ // x菉털뙤
			if((p22->m_x - cross.m_x) * (cross.m_x - p21->m_x) >= -TOLERENCE){
				P2 = true;
				if((p22->m_x - cross.m_x) * (cross.m_x - p21->m_x) <= TOLERENCE)
					P2equal0 = true;
			}
			else
				P2 = false;
		}
		else{ //y菉털뙤
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

// generate partition as 2d line segment(shoulnd't be 2d plane?)
void gb_getLoopPartition(CP_Loop& ln, vector<CP_Partition>& vp){
	//printf("\t\tgb_getLoopPartition\n"); // debug
	CP_Polygon *polygon = ln.m_polygon;
	int size = ln.m_pointIDArray.size();
	int direction = -1;
	if(ln.m_loopIDinRegion == 0)
		direction = 1;

	for(int i = 0; i < size; i++){
		int j = (i + direction + size) % size;
		//printf("\t\t\t- (i,j) = (%d, %d)\n", i, j); // debug
		CP_Partition p(polygon->m_pointArray[ln.m_pointIDArray[i]], polygon->m_pointArray[ln.m_pointIDArray[j]]);
		vp.push_back(p);
	}
}

CP_BSPNode* gb_buildPolygonBSPTree(CP_Polygon& pn){
	//printf("gb_buildPolygonBSPTree\n");
	int nr = pn.m_regionArray.size();
	if(nr == 0)
		return NULL;

	vector<CP_BSPNode *> bsptrees;
	CP_BSPNode* result = NULL;
	for(auto region : pn.m_regionArray)
		bsptrees.push_back(gb_buildRegionBSPTree(region));

	if(nr == 1) return bsptrees[0];
	else{
		result = bsptrees[0];
		for(int iR = 1; iR < nr; iR++){
			result = gb_mergeBSPTree_root(result, bsptrees[iR], CP_BSPOp::UNION);
		}
	}
	return result;
}

CP_BSPNode* gb_buildRegionBSPTree(CP_Region& rn){
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
			result = gb_mergeBSPTree_root(result, bsptrees[iL], CP_BSPOp::SUBTRACTION);
	}
	return result;
}

CP_BSPNode* gb_buildLoopBSPTree(CP_Loop& ln){
	// polygon의 에지 개수만큼..
	vector<CP_Partition> partitionArray;
	gb_getLoopPartition(ln, partitionArray);

	CP_BSPNode *tree = NULL;
	// root라서 CHILDINFO_NO를 넘김.
	printf("Start [-------------------------------------------buildBSPTree -----------------------------------------------] \n");
	tree = gb_buildBSPTree(partitionArray, NULL, CHILDINFO_NO);
	printf("End [-------------------------------------------buildBSPTree -----------------------------------------------] \n");
	return tree;
}

CP_BSPNode* gb_buildBSPTree(const vector<CP_Partition>& vp, CP_BSPNode* parent, char childInfo) {

	const CP_Partition &H = vp[0]; // H referes the H(yperplane) of current node. Just choose first one, not considering optimality......

	// 새로운 노드 생성 및 parent와 양방향 포인터 연결
	CP_BSPNode *tree = new CP_BSPNode();
	tree->partition_original = H; // hyper plane으로 node를 쪼갬.
	tree->parent = parent;
	if (childInfo == CHILDINFO_LEFT)      parent->leftChild = tree;
	else if(childInfo == CHILDINFO_RIGHT) parent->rightChild = tree;
	//else if (childInfo == CHILDINFO_NO) { printf("root node!"); }
	
	// Note : is_partition_in_region 안쪽에서 'parent'의 left, right child 포인터를 레퍼런스 하고있음
	// debug!!!!!!!!!!!!
	if (parent == NULL) { // ROOT Node
		CP_Partition partition_expanded = tree->partition_original.infinite_expansion();
		//tree->pos_coincident.push_back(partition_expanded);
		tree->partition_abstract = partition_expanded;
	}
	else {
		CP_Partition partition_splited;
		tree->is_partition_in_region(tree->partition_original, partition_splited);
		//tree->pos_coincident.push_back(partition_splited); // 해당하는 리프에서 잘려진 것을.. 다시 저장?
		//tree->partition = partition_splited; // problematic.
		tree->partition_abstract = partition_splited;

		printf("[gb_buildBSPTree] slope comp (partition, partition_splited) : (%lf, %lf)\n",
			tree->partition_original.slope(), partition_splited.slope()); // same

		printf("\t- tree->partition : (%.4lf, %.4lf) -> (%.4lf, %.4lf)\n",
			tree->partition_original.begin.m_x, tree->partition_original.begin.m_y,
			tree->partition_original.end.m_x, tree->partition_original.end.m_y);

		printf("\t- partition_splited: (%.4lf, %.4lf) -> (%.4lf, %.4lf) \n",
			partition_splited.begin.m_x, partition_splited.begin.m_y,
			partition_splited.end.m_x, partition_splited.end.m_y);
	}
	// debug!!!!!!!!!!!!

	// 현재 sub tree(노드)에 남아있는 모든 파티션(vp)들을 H에 대해서 classification 하고, H로 잘라준다.
	vector<CP_Partition> F_right, F_left;
	for (int pidx = 1; pidx < vp.size(); pidx++) {
		const CP_Partition& p = vp[pidx];
		char pos = getPartitionPos(p, H); // ClassifyPolygonToPlane / ClassifyPolygonToLine
		switch(pos){
		case POS_LEFT: // POLYGON_BEHIND_PLANE(inside)
			F_left.push_back(p);
			break;
		case POS_POS_ON: // coincident
			//tree->pos_coincident.push_back(p); // Note : it is ok not push coincidents..
			break;
		case POS_NEG_ON: // coincident
#ifdef ENABLE_BSP_NEG_COINCIDENT
			tree->neg_coincident.push_back(p);
#endif
			break;
		case POS_RIGHT: //POLYGON_IN_FRONT_OF_PLANE(outside)
			F_right.push_back(p);
			break;
		case POS_CROSS: // POLYGON_STRADDLING_PLANE
			CP_Partition left, right;
			gb_getCrossPartition(p, H, left, right); // ImplicitPolygon::Split.
			F_left.push_back(left);
			F_right.push_back(right);
			break;
		}
	}

	// H로 Tree를 Cut한 결과에 따라 하위 노드를 만들어 준다.
	// 1. 내부 (inside tree)
	if(F_left.size() == 0){ // leaf
		tree->leftChild = new CP_BSPNode();
		tree->leftChild->side = CP_BSPNode::Sideness::INSIDE;
	}
	else { // internal (sideness is undefined)
		tree->leftChild = gb_buildBSPTree(F_left, tree, CHILDINFO_LEFT);
	}
	tree->leftChild->parent = tree;

	// 1. 외부 (inside tree)
	if(F_right.size() == 0) { // leaf
		tree->rightChild = new CP_BSPNode();
		tree->rightChild->side = CP_BSPNode::Sideness::OUTSIDE;
	}
	else { // internal (sideness is undefined)		
		tree->rightChild = gb_buildBSPTree(F_right, tree, CHILDINFO_RIGHT);
	}
	tree->rightChild->parent = tree;
	/*
	printf("[-------------TreeNode Build Log----------------] Start\n");
	printf("\t- tree[%p, %d, %d] pos_coincident.size() : %d\n", tree, tree->isCell(), tree->parent == NULL, 
		tree->pos_coincident.size());

	printf("\t- tree->partition : (%.4lf, %.4lf) -> (%.4lf, %.4lf)\n",
		tree->partition.begin.m_x, tree->partition.begin.m_y,
		tree->partition.end.m_x, tree->partition.end.m_y);
	for (int pidx = 0; pidx < tree->pos_coincident.size(); pidx++) {
		const auto& pc = tree->pos_coincident[pidx];
		printf("\t- pos_coincident[%d]: (%.4lf, %.4lf) -> (%.4lf, %.4lf) \n",
			pidx,
			pc.begin.m_x, pc.begin.m_y,
			pc.end.m_x, pc.end.m_y);

	}
	printf("[-------------TreeNode Build Log----------------] End\n");
	*/
	return tree;
}

// 이미 확실히 T와 P가 cross되는 것이 보장된 상태임을 가정한다.(다시 말해서 t X p != 0)
// (outplace) T를 P로 자르고, left, right(내외부)로 파티션을 새로 생성한다.
void gb_getCrossPartition(const CP_Partition& T, const CP_Partition& P, CP_Partition& left, CP_Partition& right) 
{
	left = CP_Partition(T);
	right = CP_Partition(T);

	CP_Vec2 t_vec, p_vec;
	CP_Line2 t_line, p_line;
	CP_Point2 point = T.intersection(P, t_vec, p_vec, t_line, p_line);

	if (t_vec.cross_product(p_vec) < 0) {
		left.begin = point;
		right.end = point;
	}
	else {
		left.end = point;
		right.begin = point;
	}
}

char getPartitionPos(
	const CP_Partition &partition,   // H를 자르기 위해서 비교해야 되는 binary partitioner. ~= 'P'
	const CP_Partition &H            // 이미 생성된 BSP tree 노드의 binary partitioner. : ~= 'T'
) {
	// Naylor Figure 3.1. Check Spatial relationships between two binary partitioners.
	double begin_pos, end_pos;

	CP_Vec2 H_vector = H.end - H.begin; 

	CP_Point2 vp_begin = partition.begin;
	CP_Point2 vp_end = partition.end;

	CP_Vec2 begin_vector = vp_begin - H.begin;
	begin_pos = H_vector.cross_product(begin_vector); // check begin point coincides or...

	CP_Vec2 end_vector = vp_end - H.end;
	end_pos = H_vector.cross_product(end_vector); // check end point is coincides or...

	if(equal_float(end_pos, 0)) end_pos = 0;
	if(equal_float(begin_pos, 0)) begin_pos = 0;

	// 여기서는 두 개의 line segment의 관계에 대해서만 생각! 무한한 직선으로 생각하지 않음.
	if(end_pos * begin_pos < 0){
		return POS_CROSS;
	}
	else if(end_pos * begin_pos > 0){
		if (end_pos < 0) return POS_RIGHT;
		else return POS_LEFT;
	}
	else{ 
		// end_pos * begin_pos == 0?
		// 다시 말해 p와 H가 parallel/anti-parallel하거나, 혹은 시작점만 coincidence할 수 도 있음.
		if(end_pos < 0 || begin_pos < 0)
			return POS_RIGHT;
		else if(end_pos > 0 || begin_pos > 0)
			return POS_LEFT;
		else {
			// end_pos == 0 && begin_pos == 0 (완전 일치..)
			CP_Vec2 vp_vec = vp_end - vp_begin;
			if(vp_vec.m_x * H_vector.m_x > 0 || vp_vec.m_y * H_vector.m_y > 0)
				return POS_POS_ON;  // parallel on 
			else return POS_NEG_ON; // anti-parallel on 
		}
	}
}

CP_BSPNode* gb_mergeBSPTree_non_root(CP_BSPNode* A, CP_BSPNode* B, CP_BSPNode* parent, CP_BSPOp op, bool left){

	CP_BSPNode* tree = NULL;
	
	if(A->isCell() || B->isCell()) {
		tree = gb_mergeTreeWithCell(A, B, op);
		tree->parent = parent;
		if(left) tree->parent->leftChild = tree;
		else tree->parent->rightChild = tree;
	}
	else{
		tree = new CP_BSPNode();
		tree->parent = parent;
		tree->partition_original = A->partition_original;
		tree->partition_abstract = A->partition_abstract;
		//tree->assign_coincidents(A);  // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)

		CP_Partition partition_splited;
		B->is_partition_in_region(tree->partition_original, partition_splited);

		CP_BSPNode* B_inRight = NULL, * B_inLeft = NULL;
		gb_partitionBspt(B, tree->partition_original, B_inLeft, B_inRight, partition_splited);
		if (left) tree->parent->leftChild = tree;
		else tree->parent->rightChild = tree;
		B_inLeft->parent = tree;
		B_inRight->parent = tree;
		tree->leftChild = B_inLeft;
		tree->rightChild = B_inRight;

		gb_mergeBSPTree_non_root(A->leftChild, B_inLeft, tree, op, true);
		gb_mergeBSPTree_non_root(A->rightChild, B_inRight, tree, op, false);		

		/*
		printf("[-------------TreeNode Merge Log----------------] Start\n");
		printf("\t- tree[%p] pos_coincident.size() : %d\n", tree, tree->pos_coincident.size());

		printf("\t- tree->partition : (%.4lf, %.4lf) -> (%.4lf, %.4lf)\n",
			tree->partition.begin.m_x, tree->partition.begin.m_y,
			tree->partition.end.m_x, tree->partition.end.m_y);
		for (int pidx = 0; pidx < tree->pos_coincident.size(); pidx++) {
			const auto& pc = tree->pos_coincident[pidx];
			printf("\t- pos_coincident[%d]: (%.4lf, %.4lf) -> (%.4lf, %.4lf) \n",
				pidx,
				pc.begin.m_x, pc.begin.m_y,
				pc.end.m_x, pc.end.m_y);

		}
		printf("[-------------TreeNode Merge Log----------------] End\n");
		*/
	}

	return tree;
}

CP_BSPNode* gb_mergeBSPTree_root(CP_BSPNode* A, CP_BSPNode* B, CP_BSPOp op) {
	printf("[gb_mergeBSPTree - root node]\n");
	CP_BSPNode* new_root_node = new CP_BSPNode();
	new_root_node->partition_original = A->partition_original;
	new_root_node->partition_abstract= A->partition_abstract;
	//new_root_node->assign_coincidents(A);  // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)

	CP_BSPNode *B_inRight = NULL, *B_inLeft = NULL;
	gb_partitionBspt(
		B, new_root_node->partition_original,  // input variables.
		B_inLeft, B_inRight, // output variables.
		new_root_node->partition_original.infinite_expansion() // input variable.
	);
	B_inLeft->parent = new_root_node;
	B_inRight->parent = new_root_node;
	new_root_node->leftChild = B_inLeft;
	new_root_node->rightChild = B_inRight;

	// debug purpose... disable.
	gb_mergeBSPTree_non_root(A->leftChild, B_inLeft, new_root_node, op, true);
	gb_mergeBSPTree_non_root(A->rightChild, B_inRight, new_root_node, op, false);		
	return new_root_node;
}

CP_BSPNode* gb_mergeTreeWithCell(CP_BSPNode* T1, CP_BSPNode* T2, CP_BSPOp op){
	if(T1->isCell()){
		// Same as the Figure 5.1 in Naylor's paper.
		if(T1->side == CP_BSPNode::Sideness::INSIDE){
			switch(op){
			case CP_BSPOp::UNION:
				return T1;
			case CP_BSPOp::INTERSECTION:
				return T2;
			case CP_BSPOp::SUBTRACTION:
				T2->complement();
				return T2;
			}
		}
		else{
			switch(op){
			case CP_BSPOp::UNION:
				printf("[CP_BSPOp::UNION][T2 is %d]\n", T2->isCell());
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
		if(T2->side == CP_BSPNode::Sideness::INSIDE){
			switch(op){
			case CP_BSPOp::UNION:
				return T2;
			case CP_BSPOp::INTERSECTION:
				return T1;
			case CP_BSPOp::SUBTRACTION:
				
				CP_BSPNode *node = new CP_BSPNode();
				node->side = CP_BSPNode::Sideness::OUTSIDE;
				return node;
				
				/*
				T1->complement(); // (Q : why naylor's algorithm not working on complicate CSG tree:multiple loops are joined?)
				return T1;
				*/
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
				//return T2; // (Q : why naylor's algorithm not working on complicate CSG tree:multiple loops are joined?)
			}
		}
	}
	return NULL;
}

void gb_partitionBspt(
	const CP_BSPNode* const T, // partition할 BSP 'T'.
	const CP_Partition& partition, // BSP를 자를 원본 파티션 'P'. 
	CP_BSPNode* & B_inLeft, CP_BSPNode*& B_inRight, 
	// partition 'P'를 reqursive 하게 잘라나가는 중간 과정의 결과가 저장되는 곳.
	// A. 루트 노드에서 호출될 때는 partition line을 infinite하게 연장한 게 입력됨.
	// B. 
	const CP_Partition& splited_partition 
){ 
	// A. if T is 'cell(or leaf node)' 
	if(T->isCell()){
		B_inLeft = new CP_BSPNode(T);
		B_inRight = new CP_BSPNode(T);	
		return;
	}

	// B. if T is 'not cell(not leaf node)'
	/*
	* \brief	cross point는 T->partition, 그리고 partition 을 무한한 직선으로 생각했을 때의 교점.
	* \details	두 직선이 평행/일치할 경우 cross_point에는 쓰레기 값이 들어있게 됨. (ON_POS, ON_NEG, POS_POS, NEG_NEG).
	*/ 
	CP_Point2 cross_point; 

	// the binary partitioner of T splited by P (if intersects) is stored in partitionL(inside) and partitionR(outside).
	// if binary partitioners of T and P are 'not intersecting', then partitionL and partitionR is 'not changing'
	CP_Partition spl_partitionL, spl_partitionR;
	spl_partitionL = spl_partitionR = CP_Partition(splited_partition);

	// pos has 7 cases
	char pos = gb_t_p_Position3(T, partition, cross_point, spl_partitionL, spl_partitionR);
	switch(pos){
	case P_T_ON_POS:
		B_inLeft = T->leftChild;
		B_inRight = T->rightChild;
		//parent->pos_coincident.push_back(T->partition); // 하지 않아도 상관 없음.
		return;
	case P_T_ON_NEG:
		B_inLeft = T->rightChild;
		B_inRight = T->leftChild;
#if ENABLE_BSP_NEG_COINCIDENT == 1:
		parent->neg_coincident.push_back(T->partition);
#endif
		return;
	case P_T_POS_NEG:
		// T가 P의 Negative(Outside) 영역에 완전히 포함되는 경우..
		B_inRight = new CP_BSPNode();
		B_inRight->rightChild = T->rightChild;
		B_inRight->partition_original = T->partition_original;
		B_inRight->partition_abstract = T->partition_abstract;
		//B_inRight->assign_coincidents(T);
		gb_partitionBspt(T->leftChild, partition, B_inLeft, B_inRight->leftChild, spl_partitionL);
		break;
	case P_T_POS_POS:
		// T가 P의 Positive(Inside) 영역에 완전히 포함되는 경우..
		B_inLeft = new CP_BSPNode(); // inside니깐 left child에 연결..
		B_inLeft->rightChild = T->rightChild;
		B_inLeft->partition_original = T->partition_original;
		B_inLeft->partition_abstract = T->partition_abstract;
		//B_inLeft->assign_coincidents(T); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		gb_partitionBspt(T->leftChild, partition, B_inLeft->leftChild, B_inRight, spl_partitionL);
		break;
	case P_T_NEG_POS:
		B_inLeft = new CP_BSPNode();
		B_inLeft->leftChild = T->leftChild;
		B_inLeft->partition_original = T->partition_original;
		B_inLeft->partition_abstract = T->partition_abstract;
		//B_inLeft->assign_coincidents(T); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		gb_partitionBspt(T->rightChild, partition, B_inLeft->rightChild, B_inRight, spl_partitionR);
		break;
	case P_T_NEG_NEG:
		B_inRight = new CP_BSPNode();
		B_inRight->leftChild = T->leftChild;
		B_inRight->partition_original = T->partition_original;
		B_inRight->partition_abstract = T->partition_abstract;
		//B_inRight->assign_coincidents(T); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		gb_partitionBspt(T->rightChild, partition, B_inLeft, B_inRight->rightChild, spl_partitionR);
		break;
	case P_T_BOTH_POS:
	{
		// T와 P가 region 안에서 서로 intersection 하는 경우.
		B_inLeft = new CP_BSPNode();
		B_inLeft->partition_original = T->partition_original;
		//B_inLeft->partition_abstract = T->partition_abstract; // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		B_inRight = new CP_BSPNode();
		B_inRight->partition_original = T->partition_original;
		//B_inRight->partition_abstract = T->partition_abstract; // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)

		// Note : flipped partition insertion compared to the P_T_BOTH_NEG case.

		// T의 coincident를 partition에 대하여 다시 classification 하는 부분
		const auto& t_pc = T->partition_abstract;
		//for (const auto& t_pc : T->pos_coincident) { // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		switch (t_pc.coincidentPos(cross_point)) {
		case CP_Partition::PointSideness::LINE_IN:
		{
			CP_Partition left(t_pc.begin, cross_point);
			CP_Partition right(cross_point, t_pc.end);
			//B_inLeft->pos_coincident.push_back(left); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
			B_inLeft->partition_abstract = left;
			//B_inRight->pos_coincident.push_back(right); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
			B_inRight->partition_abstract = right;
			break;
		}
		case CP_Partition::PointSideness::LINE_POS:
			//B_inLeft->pos_coincident.push_back(t_pc); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
			B_inLeft->partition_abstract = t_pc;
			break;
		case CP_Partition::PointSideness::LINE_NEG:
			//B_inRight->pos_coincident.push_back(t_pc); // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
			B_inRight->partition_abstract = t_pc;
			break;
		}

#if ENABLE_BSP_NEG_COINCIDENT == 1:
		// enhanced for loop 으로 고치기 이전 방식은 
		// - acb922ba19585747fedf5ca63bf341f3164e7fb9 이전 커밋 (혹은 4월 7일 15:00 이전 커밋 참고)
		for (const auto& t_nc : T->neg_coincident) {
			switch (t_nc.coincidentPos(cross_point)) {
			case CP_Partition::PointSideness::LINE_IN:
			{
				CP_Partition left(cross_point, t_nc.end);
				CP_Partition right(t_nc.begin, cross_point);
				B_inLeft->neg_coincident.push_back(left);
				B_inRight->neg_coincident.push_back(right);
				break;
			}
			case CP_Partition::PointSideness::LINE_POS:
				B_inRight->neg_coincident.push_back(t_nc);
				break;
			case CP_Partition::PointSideness::LINE_NEG:
				B_inLeft->neg_coincident.push_back(t_nc);
				break;
			}
		}
#endif
		gb_partitionBspt(T->leftChild, partition, B_inLeft->leftChild, B_inRight->leftChild, spl_partitionL);
		gb_partitionBspt(T->rightChild, partition, B_inLeft->rightChild, B_inRight->rightChild, spl_partitionR);
		break;
	}
	case P_T_BOTH_NEG:
	{
		// T와 P가 region 안에서 서로 intersection 하는 경우.
		B_inLeft = new CP_BSPNode();
		B_inLeft->partition_original = T->partition_original;
		B_inRight = new CP_BSPNode();
		B_inRight->partition_original = T->partition_original;

		// T의 coincident를 partition에 대하여 다시 classification 하는 부분
		// Note : flipped partition insertion compared to the P_T_BOTH_POS case.
		const auto& t_pc = T->partition_abstract;
		//for (const auto& t_pc : T->pos_coincident) { // DEPRECATED (since that CP_BSPNode::pos_coincidence is no longer maintained)
		switch (t_pc.coincidentPos(cross_point)) {
		case CP_Partition::PointSideness::LINE_IN:
		{
			CP_Partition left(cross_point, t_pc.end);
			CP_Partition right(t_pc.begin, cross_point);
			B_inLeft->partition_abstract = left;
			B_inRight->partition_abstract = right;
			break;
		}
		case CP_Partition::PointSideness::LINE_POS:
			B_inRight->partition_abstract = t_pc;
			break;
		case CP_Partition::PointSideness::LINE_NEG:
			B_inLeft->partition_abstract = t_pc;
			break;
		}
		//}
#if ENABLE_BSP_NEG_COINCIDENT == 1:
		// enhanced for loop 으로 고치기 이전 방식은 
		// - acb922ba19585747fedf5ca63bf341f3164e7fb9 이전 커밋 (혹은 4월 7일 15:00 이전 커밋 참고)
		for (const auto& t_nc : T->neg_coincident) {
			switch (t_nc.coincidentPos(cross_point)) {
			case CP_Partition::PointSideness::LINE_IN:
			{
				CP_Partition left(cross_point, t_nc.end);
				CP_Partition right(t_nc.begin, cross_point);
				B_inLeft->neg_coincident.push_back(left);
				B_inRight->neg_coincident.push_back(right);
				break;
			}
			case CP_Partition::PointSideness::LINE_POS:
				B_inLeft->neg_coincident.push_back(t_nc);
				break;
			case CP_Partition::PointSideness::LINE_NEG:
				B_inRight->neg_coincident.push_back(t_nc);
				break;
			}
		}
#endif
		gb_partitionBspt(T->leftChild, partition, B_inLeft->leftChild, B_inRight->leftChild, spl_partitionL);
		gb_partitionBspt(T->rightChild, partition, B_inLeft->rightChild, B_inRight->rightChild, spl_partitionR);
		break;
	}
	}
	if(!B_inLeft->isCell()){
		B_inLeft->leftChild->parent = B_inLeft;
		B_inLeft->rightChild->parent = B_inLeft;
	}
	if(!B_inRight->isCell()){
		B_inRight->leftChild->parent = B_inRight;
		B_inRight->rightChild->parent = B_inRight;
	}
}

char gb_t_p_Position3(
	const CP_BSPNode* const A, const CP_Partition& partition, 
	CP_Point2& cross_point,
	CP_Partition& partitionL, CP_Partition& partitionR) {

	// [Notation]
	// 't' prefix states the 'tree'
	// 'p' prefix states the 'partition'

	const CP_Partition& t_bp = A->partition_original;

	CP_Vec2 t_vec, p_vec;
	CP_Line2 t_line, p_line;
	const CP_Point2 point_intersection = t_bp.intersection(partition, t_vec, p_vec, t_line, p_line);
	const double &ta = t_line.a, &tb = t_line.b, &tc = t_line.c;
	const double &pa = p_line.a, &pb = p_line.b, &pc = p_line.c;

	if(t_line.isParallel(p_line)){ // point intersection 계산할 때 denominator가 0인지 검사..
		// (주의) 두 직선이 평행할 때는 여기서는 교점 파라미터(cross_point)에 값이 할당되지 않음.
		CP_Vec3 cp_t_p = t_line.as_vec().cross_product(p_line.as_vec());
		if((equal_float(cp_t_p.m_x, 0) && equal_float(cp_t_p.m_y, 0)))
		{
			// [Warning from Gyu Jin Choi] : (problematic) never enters
			// intersect (coincide)
			if(ta * pa > 0 || tb * pb < 0) return P_T_ON_POS;
			else return P_T_ON_NEG;
		}
		else {
			//not intersect (real parallel)

			// 이미 P 와 T의 직선의 방정식이 parallel 하므로, p의 점이 t의 진행방향 왼쪽 공간에 있는지만 검사하면 충분함.
			CP_Point2 p_line_pt(0, -p_line.c / p_line.b);
			bool isleft_test = t_bp.is_left_side(p_line_pt);

			if (isleft_test > 0) {
				//P is to the left of T
				
				/*
				// 서로 교차하는게 아닌데.. 굳이 partition을 넣어줄 필요는 없지만, 구색을 맞추기 위해 넣음.
				partitionRBegin = partition->end; 
				partitionREnd = partition->begin;
				*/

				// 두 개의 n-hyperplane의 normal을 비교하여 서로의 위치를 최종적으로 결정함.
				// - 이 프로젝트에서는 normal을 명시적으로 저장하지 않고, 2D 직선의 방향을 이용하여 법선을 표시하고 있음에 유의할 것.
				if(ta * pa > 0 || tb * pb > 0) return P_T_POS_NEG;
				else return P_T_POS_POS; // Note : CP_Partition::intersection 안쪽에서 cross product에 들어가는 line segment의 begin, end 순서에 따라 법선 방향이 뒤집힘.
			}
			else {
				//P is to the right of T

				/*
				// 서로 교차하는게 아닌데.. 굳이 partition을 넣어줄 필요는 없지만, 구색을 맞추기 위해 넣음.
				// 필요하다면 활성화 할것.
				partitionLBegin = partition->end;
				partitionLEnd = partition->begin;
				*/

				if(ta * pa > 0 || tb * pb > 0) return P_T_NEG_POS;
				else return P_T_NEG_NEG;
			}
			
		}
	}
	else {// (tree's hyper plane T and partition P are crossInPartition)
		// t_line is intersected with p_line (infinite line is not parallel!)

		// assign output variable.
		cross_point = point_intersection; 

		///////////// Check cross_point is on the line segment (partition) ////////////
		// Note : 지금 시점에서는 (partitionL == partitionR)

		// point_intersection가 실제로 partition위에 있는 점인지 검사하는 부분, 왜냐하면 line segment위에 있는 점이 아닐 수도 있기 때문에..
		// - 삼차원에서는 직선의 방정식(line)이 polygon과 intersection 하는지 검사해야 함.
		// - partitionL, partitoinR을 알맞게 잘라서 할당해줌.

		auto coincidence = partitionL.coincidentPos(point_intersection);
		switch (coincidence) {
		case CP_Partition::PointSideness::LINE_IN: {
			const double _cp = t_vec.cross_product(p_vec);
			if (_cp > TOLERENCE) {
				partitionL.begin = point_intersection;
				partitionR.end = point_intersection;
				return P_T_BOTH_POS;
			}
			/*
			* // [ALERT] Could never be happen => already filtered by 't_line.isParallel(p_line)'
			else if (equal_float(_cp, 0)) {
				printf("[gb_t_p_Position3] in here, t and p shouldn't be parallel\n");
			}
			*/
			else { // it represents (_cp < -TOLERENCE)
				partitionR.begin = point_intersection;
				partitionL.end = point_intersection;
				return P_T_BOTH_NEG;
			}
			break;
		}
		default:
		{
			// PointSideness::LINE_NEG or PointSideness::LINE_POS
			//inside and disjoint

			// Calculate the p direction
			// 걍 기울기에 따라서..
			double a, b;
			if (std::abs(pa) > std::abs(pb)) {
				//y방향
				a = std::abs(partitionL.begin.m_y - point_intersection.m_y);
				b = std::abs(partitionL.end.m_y - point_intersection.m_y);
			}
			else {
				//x방향
				a = std::abs(partitionL.begin.m_x - point_intersection.m_x);
				b = std::abs(partitionL.end.m_x - point_intersection.m_x);
			}

			double dirP = a - b;
			if (dirP > 0) dirP = 1;
			else dirP = -1;			

			// Calculate the t direction
			if (std::abs(ta) > std::abs(tb)) {
				//y방향
				// A node의 현재 binary partition(원본 메쉬에서 온 것 말고, BSP 빌드 과정에서 생성된, infinite 하게 연장될 수 있는 것.)
				a = std::abs(A->partition_abstract.begin.m_y - point_intersection.m_y);
				b = std::abs(A->partition_abstract.end.m_y - point_intersection.m_y);
			}
			else {
				//x방향
				a = std::abs(A->partition_abstract.begin.m_x - point_intersection.m_x);
				b = std::abs(A->partition_abstract.end.m_x - point_intersection.m_x);
			}

			double dirAP = a - b;
			if (dirAP > 0) dirAP = 1; // LINE_POS
			else dirAP = -1;		  // LINE_NEG
			
			printf("dirAP * dirP : %lf, coincidence : %d\n", dirAP * dirP, coincidence == CP_Partition::PointSideness::LINE_POS);

			CP_Partition _p(partition.end, partition.begin); // temporary object for assignment.
			if (partition.is_left_side(A->partition_abstract.begin)) {
				if (dirAP * dirP < 0) {
					partitionR = _p;
					return P_T_POS_POS;
				}
				else {
					partitionL = _p;
					return P_T_NEG_POS;
				}
			}
			else {
				if (dirAP * dirP < 0) {
					partitionL = _p;
					return P_T_NEG_NEG;
				}
				else {
					partitionR = _p;
					return P_T_POS_NEG;
				}
			}
			break;
		}
		}
	}
}

void debugBsptree(CP_BSPNode* T){
	char filename[] = "debug.txt";
	std::ofstream fout;
	fout.open(filename);
	_debugFoutBsptree(T, 0, fout);
	fout.close();
}

void _debugFoutBsptree(CP_BSPNode* T, int floor, ofstream &fout){
	char *str = new char[floor + 1];
	for(int i = 0; i < floor; i++)
		str[i] = ' ';
	str[floor] = 0;
	if(T->side == CP_BSPNode::Sideness::UNDEFINED){
		/*
		fout<<str<<"("<<T->partition.begin.m_x<<","<<T->partition.begin.m_y<<")---->("<<T->partition.end.m_x<<","<<T->partition.end.m_y<<")"<<endl;
		_debugFoutBsptree(T->leftChild, floor + 3, fout);
		_debugFoutBsptree(T->rightChild, floor + 3, fout);*/
	}
	else{	
		if(T->side == CP_BSPNode::Sideness::INSIDE)
			fout<<str<<"IN"<<endl;
		else
			fout<<str<<"OUT"<<endl;
	}
	delete[] str;
}

bool gb_generateCellPolygon(CP_BSPNode *cell){
	// generate polygons from cell

	CP_BSPNode *node = cell;
	CP_BSPNode *child = cell;
	while(node->parent != NULL){
		child = node;
		node = node->parent;

		CP_Partition p(node->partition_original); // build에 사용된 원본..
		bool no_useful = false;
		for(auto& cell_polygon : cell->polygon){
			if(!gb_cutPolygonFace(p, cell_polygon)){
				no_useful = true;
				break;
			}
		}
		
		if(!no_useful){
			//Determine whether it contributes to the polygon of the node
			if(child == node->rightChild){
				if(cell->side == CP_BSPNode::Sideness::INSIDE)
					node->rightIn.push_back(p);
				else
					node->rightOut.push_back(p);
			}
			else{
				if(cell->side == CP_BSPNode::Sideness::INSIDE)
					node->leftIn.push_back(p);
				else
					node->leftOut.push_back(p);
			}
		}


#if ENABLE_BSP_NEG_COINCIDENT
		for(unsigned int i = 0; i < node->neg_coincident.size(); i++){
			CP_Partition* p = new CP_Partition(node->neg_coincident[i]);

			bool no_useful = false;
			for(unsigned int i = 0; i < cell->polygon.size(); i++){
				CP_Partition *face = cell->polygon[i];
				if(!gb_cutPolygonFace(p, face)){
					no_useful = true;
					break;
				}
			}

			if(!no_useful){
				//Determine whether it contributes to the polygon of the node
				CP_Partition *node_face = new CP_Partition(*p);
				if(child == node->rightChild){
			
					if(cell->side == CP_BSPNode::Sideness::INSIDE){
						node->rightIn.push_back(node_face);
					}
					else{
						node->rightOut.push_back(node_face);
					}

				}
				else{
					if(cell->side == CP_BSPNode::Sideness::INSIDE){
						node->leftIn.push_back(node_face);
					}
					else{
						node->leftOut.push_back(node_face);
					}
				}
			}
		}
#endif
	}
	return true;
}

bool gb_generateCellPolygonPre(CP_BSPNode *cell)
{
	CP_BSPNode *node = cell;
	CP_BSPNode *child = cell;

	while(node->parent != NULL){
		//generate polygon
		child = node;
		node = node->parent;

		//Determine whether it contributes to the shape of the restricted cell polygon	
		CP_Partition polygon_face(node->partition_original);
		if(gb_p_in_cellPolygon(cell, polygon_face)) {
			if(child == node->rightChild)
				polygon_face.flip();

			cell->polygon.push_back(polygon_face);
		}
	}
	return true;
}

bool gb_generateCellPolygons(CP_BSPNode *node){
	if(node->isCell()){
		gb_generateCellPolygonPre(node);
		gb_generateCellPolygon(node);
		return true;
	}
	gb_generateCellPolygons(node->leftChild);
	gb_generateCellPolygons(node->rightChild);
	return true;
}

// p를 f로 잘라버린다.(inplace)
bool gb_cutPolygonFace(CP_Partition &p, const CP_Partition &face){

	CP_Vec2 face_vec = face.end - face.begin;
	CP_Vec2 begin_vec = p.begin - face.end;
	CP_Vec2 end_vec = p.end - face.end;
	double begin = face_vec.cross_product(begin_vec);
	double end = face_vec.cross_product(end_vec);

	if(abs(begin) <= TOLERENCE) // begin 이 0보다 작으면?
		begin = 0;
	if(abs(end) <= TOLERENCE)// end가 0보다 작으면?
		end = 0;

	if(begin * end < 0){
		//cut
		const CP_Partition& t_bp = face;

		CP_Vec2 t_vec, p_vec;
		CP_Line2 t_line, p_line;
		const CP_Point2 point_intersection = t_bp.intersection(p, t_vec, p_vec, t_line, p_line);
		const double& ta = t_line.a, & tb = t_line.b, & tc = t_line.c;
		const double& pa = p_line.a, & pb = p_line.b, & pc = p_line.c;

		if(begin < 0)
			p.begin = point_intersection;
		else
			p.end = point_intersection;
		return true;
	}
	else if(begin + end > 0)
		return true;
	else if(begin + end < 0){
		return false;
	}
	else{
		// The position of p coincides with the straight line of face
		return true;
	}
}

bool gb_generateBSPTreeFaces(CP_BSPNode *node){
	if(node->isCell())
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
		for(const CP_Partition& p : node->leftIn){
			for(const CP_Partition& f : node->rightOut){
				CP_Partition result;
				if(gb_cutParallelFace(p, f, result))
					node->polygon.push_back(result);
			}
		}
	}

	if(node->leftOut.size() * node->rightIn.size() != 0){
		for (const CP_Partition& p : node->leftOut) {
			for (const CP_Partition& f : node->rightIn) {
				CP_Partition result;
				if(gb_cutParallelFace(p, f, result))
					node->polygon.push_back(result);
			}
		}
	}
	return true;
}

bool gb_cutParallelFace(const CP_Partition &p, const CP_Partition &face, CP_Partition &result){
	CP_Vec2 face_vec = face.end - face.begin;
	int dx = face_vec.m_x < 0 ? -1 : 1;
	int dy = face_vec.m_y < 0 ? -1 : 1;

	double face_begin = 0;
	double face_end = std::abs(face_vec.m_x) + std::abs(face_vec.m_y);

	// what does this even mean?
	CP_Vec2 begin_vec = p.begin - face.begin;
	CP_Vec2 end_vec = p.end - face.begin;
	double p_begin = begin_vec.m_x * dx + begin_vec.m_y * dy;
	double p_end = end_vec.m_x * dx + end_vec.m_y * dy;

	if(p_end <= face_begin || p_begin >= face_end)
		return false;
	else if(p_begin <= face_begin && p_end >= face_end){
		result = face;
		return true;
	}
	else if(p_begin >= face_begin && p_end <= face_end){
		result = p;
		return true;
	}
	else if(p_begin <= face_begin && p_end > face_begin){
		result = CP_Partition(face.begin, p.end);
		return true;
	}
	else{
		result = CP_Partition(p.begin, face.end);
		return true;
	}
}

//is_partition_in_region dl
bool gb_p_in_cellPolygon(
	const CP_BSPNode* const T, const CP_Partition& partition
){
	// [직선의 방정식의 steepest-axis 찾기] Start
	// - 왜냐하면, 어디서 잘라야 하는지 저장할 때, vector가 X, Y축에 parallel 할 수 있기 때문에
	// - 더 긴 쪽으로 하기 위함..
	const CP_Vec2 diff = partition.end - partition.begin;
	const double& dx = diff.m_x, & dy = diff.m_y;
	const double mean_xy[2] = {
		dx > 0 ? 1 : -1,
		dy > 0 ? 1 : -1
	};
	const bool x_or_y = std::abs(dx) < std::abs(dy) ? true : false; // dx, dy 중어느 것이 더 큰지 검사.
	// [직선의 방정식의 steepest-axis 찾기] End

	// [!!!!!!!!!!!주의!!!!!!!!!] extension 이 너무 크면 계산 오류가 있음.
	double min = DBL_MAX * -1; 
	double max = DBL_MAX;

	for(unsigned int i = 0; i < T->polygon.size(); i++){
		const CP_Partition& t_bp = T->polygon[i];

		CP_Vec2 t_vec, p_vec;
		CP_Line2 t_line, p_line;
		CP_Point2 point = t_bp.intersection(partition, t_vec, p_vec, t_line, p_line);
		double cross_product_tp = t_vec.cross_product(p_vec); // --- (1) t_bp에 영향을 받는데..

		if (equal_float(cross_product_tp, 0)) {

			CP_Vec2 v = partition.begin - t_bp.begin;
			if (t_vec.cross_product(v) >= 0) {
				continue;
			}
			else{
				return false;
			}
		}
		// 만약 두 개의 벡터가 평행하지 않은 경우...
		else if (cross_product_tp > TOLERENCE)
		{
			// p 벡터가 t벡터에 대해서 CCW 방향으로 rotation 되어있을 경우 (1)번 cross product 참고
			double currentMin = !x_or_y ?
				(point.m_x - partition.begin.m_x) * mean_xy[x_or_y] : // == 0
				(point.m_y - partition.begin.m_y) * mean_xy[x_or_y];  // == 1

			if (currentMin >= max) {
				return false;
			}
			else
				if (currentMin > min) {
					min = currentMin;
				}
		}
		else { // (cross_product_tp < -TOLERENCE)
			// p 벡터가 t벡터에 대해서 CW 방향으로 rotation 되어있을 경우 (1)번 cross product 참고
			double currentMax = !x_or_y ?
				(point.m_x - partition.begin.m_x) * mean_xy[x_or_y] : // == 0
				(point.m_y - partition.begin.m_y) * mean_xy[x_or_y];  // == 1

			if (currentMax <= min) {
				return false;
			}
			else
				if (currentMax < max) {
					max = currentMax;
				}
		}
	}

	return true;	
}


/*
char gb_t_p_Position(CP_BSPNode* A, CP_Partition* partition, CP_Point2 &cross_point, CP_Point2& partitionLBegin, CP_Point2& partitionLEnd, CP_Point2& partitionRBegin, CP_Point2& partitionREnd){
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
		else{  //꼇宮슥
			double isleft = -tb * (partition->end.m_y - t_bp->end.m_y) -ta * (partition->end.m_x - t_bp->end.m_x);
			if(isleft > 0){ //P瞳T璘긋
				partitionRBegin = partition->end;
				partitionREnd = partition->begin;
				if(ta * pa > 0 || tb * pb > 0){
					return P_T_POS_NEG;
				}
				else{
					return P_T_POS_POS;
				}
			}
			else{//P瞳T塘긋
				partitionLBegin = partition->end;
				partitionLEnd = partition->begin;
				if(ta * pa > 0 || tb * pb > 0){
					return P_T_NEG_POS;
				}
				else{
					return P_T_NEG_NEG;
				}
			}
		}
	}
	else{//宮슥
		//헹t뵨p돨슥듐
		CP_Point2 point;
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
		cross_point = CP_Point2();
		cross_point = point;

		//털뙤슥듐角뤠瞳partitionBegin뵨partitionEnd코꼬
		double dx = partitionLEnd.m_x - partitionLBegin.m_x;
		double dy = partitionLEnd.m_y - partitionLBegin.m_y;
		bool crossInpartition = false;
		if(pa * pa > pb * pb){ //y렘蕨
			if(((partitionLEnd.m_y - point.m_y > TOLERENCE) && (point.m_y - partitionLBegin.m_y > TOLERENCE))
				|| ((partitionLEnd.m_y - point.m_y < -TOLERENCE) && (point.m_y - partitionLBegin.m_y < -TOLERENCE))){ //in
				crossInpartition = true;
			}
			if(partitionLEnd.m_y / pa - partitionLBegin.m_y / pa < 0)
				crossInpartition = false;
		}
		else{//x렘蕨
			if(((partitionLEnd.m_x - point.m_x > TOLERENCE) && (point.m_x - partitionLBegin.m_x > TOLERENCE))
				|| ((partitionLEnd.m_x - point.m_x < -TOLERENCE) && (point.m_x - partitionLBegin.m_x < -TOLERENCE))){ //in
				crossInpartition = true;
			}
			if(partitionLEnd.m_x / (-pb) - partitionLBegin.m_x / (-pb) < 0)
				crossInpartition = false;
		}


		if(crossInpartition){
			if((-tb) * pa - (-pb) * ta > 0){
				partitionLBegin = point;
				partitionREnd = point;
				return P_T_BOTH_POS;
			}
			else{
				partitionRBegin = point;
				partitionLEnd = point;
				return P_T_BOTH_NEG;
			}
		}
		else{
			// 털뙤partition角뤠唐옵콘셨崎관벵瞳혐堵櫓돨窟뙈
			CP_Point2 begin, end;
			double pmin, pmax, pcross;

			if(gb_p_in_region(A, partition, begin, end, point, pmin, pmax, pcross)){

				CP_Partition *t_partition = new CP_Partition();
				t_partition->begin = A->partition->begin;
				t_partition->end = A->partition->end;

				CP_Point2 pos_point;
				double tmin, tmax, tcross;
				gb_t_in_region(A, t_partition, pos_point, &point, tmin, tmax, tcross);

				CP_Partition *currentp = new CP_Partition();
				currentp->begin = begin;
				currentp->end = end;

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
						partitionRBegin = partition->end;
						partitionREnd = partition->begin;
						return P_T_POS_POS;
					}
					else{
						partitionLBegin = partition->end;
						partitionLEnd = partition->begin;
						return P_T_NEG_POS;
					}
				}
				else{
					if(dirAP * dirP < 0){
						partitionLBegin = partition->end;
						partitionLEnd = partition->begin;
						return P_T_NEG_NEG;
					}
					else{
						partitionRBegin = partition->end;
						partitionREnd = partition->begin;
						return P_T_POS_NEG;
					}
				}
			}
			else{
				partitionLBegin = partition->end;
				partitionLEnd = partition->begin;
				partitionRBegin = partition->end;
				partitionREnd = partition->begin;
				if(gb_t_p_left(A->partition, partition)) return P_T_NEG_POS;
				else return P_T_NEG_NEG;
			}
		}
	}

}
*/


/*
bool gb_t_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point2 &pos, CP_Point2 *cross,
	double &pmin, double &pmax, double &pcross){
	pos = partition->begin;
	double vx = partition->end.m_x - partition->begin.m_x;
	double vy = partition->end.m_y - partition->begin.m_y;

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
	CP_Point2 point;
	while(node->parent != NULL){
		child = node;
		node = node->parent;
		CP_Partition* t_bp = new CP_Partition();

		if(child == node->leftChild){
			t_bp->begin = node->partition->begin;
			t_bp->end = node->partition->end;
		}
		else{
			t_bp->begin = node->partition->end;
			t_bp->end = node->partition->begin;
		}
		//CP_Partition* t_bp = node->partition;
		ta =t_bp->end.m_y - t_bp->begin.m_y;
		tb =t_bp->begin.m_x - t_bp->end.m_x;
		tc = -ta * t_bp->begin.m_x - tb * t_bp->begin.m_y;

		pa =partition->end.m_y - partition->begin.m_y;
		pb =partition->begin.m_x - partition->end.m_x;
		pc = - pa * partition->begin.m_x - pb * partition->begin.m_y;

		if((-tb) * pa - (-pb) * ta >= -TOLERENCE && (-tb) * pa - (-pb) * ta  <= TOLERENCE){
			//틱契 君瞳솝땍路북샀諒틱契瞳Tnode-partition璘긋떼옵鹿
			// node-partition蕨좆（-tb,ta）
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
*/