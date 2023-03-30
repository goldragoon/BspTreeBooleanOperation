// this project
#include "stdafx.h"
#include "CP_Polygon.h"

// std stuffs
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

bool compare_float(double x, double y, double epsilon = TOLERENCE) {
	if (fabs(x - y) < epsilon)
		return true; //they are same
	return false; //they are not same
}

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

void gb_insertPointInPolygon(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, CP_Point& newPoint)
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
void gb_pointConvertFromGlobalToScreen(CP_Point& result, CP_Point pointGlobal, double scale, CP_Point translation, int screenX, int screenY)
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
void gb_pointConvertFromScreenToGlobal(CP_Point& result, CP_Point pointScreen, double scale, CP_Point translation, int screenX, int screenY)
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
    } // for(iLv)써監
    pn.m_regionArray[idRegion].m_loopArray.erase(
        pn.m_regionArray[idRegion].m_loopArray.begin( )+idLoop);
    return true;
} // 변鑒gb_removeLoop써監

bool gb_removePoint(CP_Polygon& pn, int id)
{
    int ir, iL, iLv, nLv;
    bool rf = gb_findPointInLoop(pn, ir, iL, iLv, id);
    if (!rf)
        return false;
    nLv = pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.size( );
    if (nLv<4) // 뇜憐몸뻔
        return (gb_removeLoop(pn, ir, iL));
    pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.erase(
        pn.m_regionArray[ir].m_loopArray[iL].m_pointIDArray.begin( )+iLv);
    pn.m_pointArray.erase(pn.m_pointArray.begin( )+id);
    gb_subtractOneAboveID(pn, id);
    return true;
} // 변鑒gb_removePoint써監

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
        } // for(iLv)써監
    } // for(iL)써監
    pn.m_regionArray.erase(pn.m_regionArray.begin( )+idRegion);
    return true;
} // 변鑒gb_removeRegion써監

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
//북랬昑쇱꿴
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
	CP_Point *p0, *p1, *p2;
	int v1, v2;

	//털뙤角뤠槨糠珂濾
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

	//털뙤xmin듐杰젯窟뙈角뤠宅페儉窟뙈宮슥
	double a0, b0, c0, a, b, c;
	a0 = pmin.m_y - pminl.m_y;
	b0 = pminl.m_x - pmin.m_x;
	c0 = pmin.m_y * (pmin.m_x - pminl.m_x) - pmin.m_x * (pmin.m_y - pminl.m_y);
	for(int i = 0; i < np; i++){
		if(i == minl || i == min || i == (minl - 1 + np) % np)
			continue;
		CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
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
		CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
		CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
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
			CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
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
			CP_Point &pl1 = pn->m_pointArray[ln.m_pointIDArray[i]];
			CP_Point &pl2 = pn->m_pointArray[ln.m_pointIDArray[(i + 1) % np]];
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

//닸瞳宮슥돨긋橙럿쀼true，뤠橙럿쀼fasle
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

//털뙤寧몸bsptree櫓角뤠唐in돨秊綾쌘듐
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

//宮슥앎럿쀼true，뤠橙false
bool gb_checkLineSegmentCross(CP_Point* p11, CP_Point* p12, CP_Point* p21, CP_Point* p22){
	double ta, tb, tc, pa, pb, pc;
	CP_Point cross;
	//횅땍殮窟렘넋
	pa = p11->m_y - p12->m_y;
	pb = p12->m_x - p11->m_x;
	pc = p11->m_y * (p11->m_x - p12->m_x) - p11->m_x * (p11->m_y - p12->m_y);
	//횅땍殮窟렘넋
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
void gb_getLoopPartition(CP_Loop& ln, vector<CP_Partition*>& vp){
	printf("\t\tgb_getLoopPartition\n"); // debug
	CP_Polygon *polygon = ln.m_polygon;
	int size = ln.m_pointIDArray.size();
	int direction = -1;
	if(ln.m_loopIDinRegion == 0)
		direction = 1;

	for(int i = 0; i < size; i++){
		int j = (i + direction + size) % size;
		printf("\t\t\t- (i,j) = (%d, %d)\n", i, j); // debug
		CP_Partition* p = new CP_Partition(polygon->m_pointArray[ln.m_pointIDArray[i]], polygon->m_pointArray[ln.m_pointIDArray[j]]);
		vp.push_back(p);
	}
}

CP_BSPNode* gb_buildPolygonBSPTree(CP_Polygon& pn){
	printf("gb_buildPolygonBSPTree\n");
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

	// polygon의 에지 개수만큼..
	vector<CP_Partition*> partitionArray;
	gb_getLoopPartition(ln, partitionArray);

	CP_BSPNode *tree = NULL;
	// root라서 CHILDINFO_NO를 넘김.
	tree = gb_buildBSPTree(partitionArray, NULL, CHILDINFO_NO);
	return tree;
}

CP_BSPNode* gb_buildBSPTree(vector<CP_Partition*> &vp, CP_BSPNode* parent, char childInfo) {
	vector<CP_Partition*> F_right;
	vector<CP_Partition*> F_left;
	//vector<CP_Partition*> F_coincident; need it?

	CP_Partition *H = vp[0]; // H referes the H(yperplane) of current node. Just choose first one, not considering optimality......

	// 새로운 노드 생성.
	CP_BSPNode *tree = new CP_BSPNode();
	tree->partition = H; // hyper plane으로 node를 쪼갬.
	tree->parent = parent;

	// parent의 child node pointer를 update하기..
	if (childInfo == CHILDINFO_LEFT)      parent->leftChild = tree;
	else if(childInfo == CHILDINFO_RIGHT) parent->rightChild = tree;
	// else (childInfo == CHILDINFO_NO) 인 경우는 root node일 때밖에 없음.

	// partitionLine is used to record the part of the line where the partition is located inside the area
	CP_Partition * partitionLine = new CP_Partition(*(tree->partition));

	// temporary variables for gb_p_in_region
	CP_Point pBegin, pEnd;
	double pmin, pmax, pcross;
	CP_Point point;

	// 아래 호출에서 pcross는 쓰레기값.
	if(!gb_p_in_region(tree, partitionLine, pBegin, pEnd, point, pmin, pmax, pcross)){
		pBegin = tree->partition->end;
		pEnd = tree->partition->begin;
	}
	else{
		CP_Vec2 diff = tree->partition->end - tree->partition->begin;
		double &dx = diff.m_x, &dy = diff.m_y;

		double mean_xy[2];
		mean_xy[0] = dx > 0 ? 1: -1;
		mean_xy[1] = dy > 0 ? 1: -1;

		int x_or_y = 0; // manitude comparison?
		if(dx * dx < dy * dy)
			x_or_y = 1;

		if(x_or_y == 0){ // ||dx|| > ||dy||
			pBegin.m_x = pmin * mean_xy[0] + tree->partition->begin.m_x;
			pEnd.m_x = pmax * mean_xy[0] + tree->partition->begin.m_x;
			pBegin.m_y = (pBegin.m_x - tree->partition->begin.m_x) * (dy / dx) + tree->partition->begin.m_y;
			pEnd.m_y = (pEnd.m_x - tree->partition->begin.m_x) * (dy / dx) + tree->partition->begin.m_y;
		}
		else{
			pBegin.m_y = pmin * mean_xy[1] + tree->partition->begin.m_y;
			pEnd.m_y = pmax * mean_xy[1] + tree->partition->begin.m_y;
			pBegin.m_x = (pBegin.m_y - tree->partition->begin.m_y) * (dx / dy) + tree->partition->begin.m_x;
			pEnd.m_x = (pEnd.m_y - tree->partition->begin.m_y) * (dx / dy) + tree->partition->begin.m_x;
		}
	}

	partitionLine->begin = pBegin;
	partitionLine->end = pEnd;

	tree->pos_coincident.push_back(partitionLine);
	//partitionLine initialization ends

	if(vp.size() > 0) // Q : 이 조건이 H assign 하지 전으로 들어가야 하지 않나?
		tree->pos_coincident.push_back(H); // 현재 노드의 hyperplane (이게 partitionLine push_back 하기 전에 들어가면 왜 문제가 되나?)

	// 현재 sub tree(노드)에 남아있는 모든 파티션들을 H에 classification 하고, H로 잘라준다.
	for(const auto &p : vp){
		char pos = getPatitionPos(p, H);
		switch(pos){
		case POS_LEFT:
			F_left.push_back(p);
			break;
		case POS_POS_ON:
			tree->pos_coincident.push_back(p);
			break;
		case POS_NEG_ON:
			tree->neg_coincident.push_back(p);
			break;
		case POS_RIGHT:
			F_right.push_back(p);
			break;
		case POS_CROSS:
			CP_Partition *left = NULL;
			CP_Partition * right = NULL;
			gb_getCrossPartition(p, H, left, right);
			F_left.push_back(left);
			F_right.push_back(right);
			break;
		}
	}

	// H로 Tree를 Cut한 결과에 따라 하위 노드를 만들어 준다.
	if(F_left.size() == 0){
		tree->leftChild = new CP_BSPNode();
		tree->leftChild->position = REGION_IN;
		tree->leftChild->parent = tree;
	}
	else {
		tree->leftChild = gb_buildBSPTree(F_left, tree, CHILDINFO_LEFT);
		tree->leftChild->parent = tree;
	}

	if(F_right.size() == 0){
		tree->rightChild = new CP_BSPNode();
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
	left = new CP_Partition(T);
	right = new CP_Partition(T);

	CP_Vec2 t = T->end - T->begin;

	double pa, pb, pc, ta, tb, tc;
	ta =T->end.m_y - T->begin.m_y;
	tb =T->begin.m_x - T->end.m_x;
	tc = -ta * T->begin.m_x - tb * T->begin.m_y;

	pa =P->end.m_y - P->begin.m_y;
	pb =P->begin.m_x - P->end.m_x;
	pc = -pa * P->begin.m_x - pb * P->begin.m_y;

	CP_Point point;
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

char getPatitionPos(const CP_Partition* const partition, const CP_Partition* const H) {
	double begin_pos, end_pos;

	CP_Vec2 H_vector = H->end - H->begin;

	CP_Point vp_begin = partition->begin;
	CP_Point vp_end = partition->end;

	CP_Vec2 begin_vector = vp_begin - H->begin;
	begin_pos = H_vector.cross_product(begin_vector);

	CP_Vec2 end_vector = vp_end - H->end;
	end_pos = H_vector.cross_product(end_vector);

	if(compare_float(end_pos, 0)) end_pos = 0;
	if(compare_float(begin_pos, 0)) begin_pos = 0;

	if(end_pos * begin_pos < 0){
		return POS_CROSS;
	}
	else if(end_pos * begin_pos > 0){
		if (end_pos < 0)return POS_RIGHT;
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
			if(a1 * H_vector.m_x > 0 || b1 * H_vector.m_y >0)
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
		tree->parent = parent;
		tree->partition = A->partition;
		tree->assign_coincidents(A);

		CP_Point pBegin, pEnd;
		double pmin, pmax, pcross;
		CP_Point point;
		// 아래 호출에서 pcross는 쓰레기값.
		if(!gb_p_in_region(B, A->partition, pBegin, pEnd, point, pmin, pmax, pcross)){
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
		tree->partition = A->partition;
		tree->assign_coincidents(A);

		CP_Vec2 pDiff = tree->partition->end - tree->partition->begin;
		pDiff.normalize();

		CP_Vec2 sub(pDiff * (DBL_MAX / 2));
		CP_Point pBegin(tree->partition->begin - sub);
		CP_Point pEnd(tree->partition->begin + sub);

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
				node->position = REGION_OUT;
				return node;
				
				/*
				gb_complement(T1); // (Q : why naylor's algorithm not working on complicate CSG tree:multiple loops are joined?)
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

void gb_partitionBspt(CP_BSPNode* T, CP_Partition* partition, CP_BSPNode* & B_inLeft, CP_BSPNode*& B_inRight, CP_BSPNode* root, CP_Point& partitionBegin, CP_Point& partitionEnd){
	// if T is 'cell(or leaf node)' 
	if(gb_treeIsCell(T)){
		B_inLeft = new CP_BSPNode(T);
		B_inRight = new CP_BSPNode(T);	
		return;
	}

	// if T is 'not cell(not leaf node)'
	CP_Point cross_point;
	CP_Partition *partitionPush = NULL;
	CP_Partition *leftPartition = partition;
	CP_Partition *rightPartition = partition;
	CP_Point pLBegin, pLEnd, pRBegin, pREnd;
	pLBegin = partitionBegin;
	pLEnd = partitionEnd;
	pRBegin = partitionBegin;
	pREnd = partitionEnd;

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
		B_inRight->rightChild = T->rightChild;
		B_inRight->partition = T->partition;
		B_inRight->assign_coincidents(T);
		gb_partitionBspt(T->leftChild, partition, B_inLeft, B_inRight->leftChild, root, pLBegin, pLEnd);
		break;
	case P_T_POS_POS:
		B_inLeft = new CP_BSPNode();
		B_inLeft->rightChild = T->rightChild;
		B_inLeft->partition = T->partition;
		B_inLeft->assign_coincidents(T);
		gb_partitionBspt(T->leftChild, partition, B_inLeft->leftChild, B_inRight, root, pLBegin, pLEnd);
		break;
	case P_T_NEG_POS:
		B_inLeft = new CP_BSPNode();
		B_inLeft->leftChild = T->leftChild;
		B_inLeft->partition = T->partition;
		B_inLeft->assign_coincidents(T);
		gb_partitionBspt(T->rightChild, partition, B_inLeft->rightChild, B_inRight, root, pRBegin, pREnd);
		break;
	case P_T_NEG_NEG:
		B_inRight = new CP_BSPNode();
		B_inRight->leftChild = T->leftChild;
		B_inRight->partition = T->partition;
		B_inRight->assign_coincidents(T);
		gb_partitionBspt(T->rightChild, partition, B_inLeft, B_inRight->rightChild, root, pRBegin, pREnd);
		break;
	case P_T_BOTH_POS:
		B_inLeft = new CP_BSPNode();
		B_inRight = new CP_BSPNode();
		B_inRight->partition = T->partition;
		B_inLeft->partition = T->partition;

		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->pos_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();

				left->begin = T->pos_coincident[i]->begin;
				left->end = cross_point;
				right->begin = cross_point;
				right->end = T->pos_coincident[i]->end;
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

				left->begin = cross_point;
				left->end = T->neg_coincident[i]->end;
				right->begin = T->neg_coincident[i]->begin;
				right->end = cross_point;
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
		B_inRight = new CP_BSPNode();
		B_inRight->partition = T->partition;
		B_inLeft->partition = T->partition;

		for(unsigned int i = 0; i < T->pos_coincident.size(); i++){
			CP_Partition *right = NULL;
			CP_Partition *left = NULL;
			switch(gb_coincidentPos(T->pos_coincident[i], cross_point)){
			case LINE_IN:				
				right = new CP_Partition();
				left = new CP_Partition();

				left->begin = cross_point;
				left->end = T->pos_coincident[i]->end;
				right->begin = T->pos_coincident[i]->begin;
				right->end = cross_point;
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

				left->begin = T->neg_coincident[i]->begin;
				left->end = cross_point;
				right->begin = cross_point;
				right->end = T->neg_coincident[i]->end;

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

/*
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
		CP_Point point;
		point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
		point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
		cross_point = CP_Point();
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
			CP_Point begin, end;
			double pmin, pmax, pcross;
			
			if(gb_p_in_region(A, partition, begin, end, point, pmin, pmax, pcross)){
				
				CP_Partition *t_partition = new CP_Partition();
				t_partition->begin = A->partition->begin;
				t_partition->end = A->partition->end;

				CP_Point pos_point;
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

	bool not_in_region = false; // ??
	if(pa * pa > pb * pb){ // y방향
		if(partitionLEnd.m_y / pa - partitionLBegin.m_y / pa < 0)
			not_in_region = true;
	}
	else{ // x방향
		if(partitionLEnd.m_x / (-pb) - partitionLBegin.m_x / (-pb) < 0)
			not_in_region = true;
	}

	if(not_in_region){
		if (gb_t_p_left(A->partition, partition)) return P_T_NEG_POS;
		else return P_T_NEG_NEG;
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
		else{//殮窟宮슥
			CP_Point point;
			point.m_x =  (-tc * pb + tb * pc) / (ta * pb - tb * pa);
			point.m_y =  (tc * pa - ta * pc) / (ta * pb - tb * pa);
			cross_point = CP_Point();
			cross_point = point;
			bool crossInpartition = false;
			if(pa * pa > pb * pb){ //y렘蕨
				if(((partitionLEnd.m_y - point.m_y > TOLERENCE) && (point.m_y - partitionLBegin.m_y > TOLERENCE))
					|| ((partitionLEnd.m_y - point.m_y < -TOLERENCE) && (point.m_y - partitionLBegin.m_y < -TOLERENCE))){ //in
					crossInpartition = true;
				}
			}
			else{//x렘蕨
				if(((partitionLEnd.m_x - point.m_x > TOLERENCE) && (point.m_x - partitionLBegin.m_x > TOLERENCE))
					|| ((partitionLEnd.m_x - point.m_x < -TOLERENCE) && (point.m_x - partitionLBegin.m_x < -TOLERENCE))){ //in
					crossInpartition = true;
				}
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
			else{//혐堵코할꼇宮슥
				CP_Point begin, end;

				//셕炬p렘蕨
				double a, b;
				if(pa * pa > pb * pb){ //y렘蕨
					a = partitionLBegin.m_y - point.m_y;
					b = partitionLEnd.m_y - point.m_y;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				else{//x렘蕨
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
				
				////셕炬t렘蕨
				if(A->pos_coincident.size() == 0)
					int baa = 1;
				if(ta * ta > tb * tb){ //y렘蕨
					a = A->pos_coincident[0]->begin.m_y - point.m_y;
					b = A->pos_coincident[0]->end.m_y - point.m_y;
					if(a < 0) a *= -1;
					if(b < 0) b *= -1;
				}
				else{//x렘蕨
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
		}
	}	
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

// partition 이 T의 내부 영역에 존재하는지 검사한다.
bool gb_p_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point& end, const CP_Point &cross, 
	double &pmin, double &pmax, double &pcross){
	begin = partition->begin;
	end = partition->end;

	// diff
	double vx = partition->end.m_x - partition->begin.m_x;
	double vy = partition->end.m_y - partition->begin.m_y;
	
	// 사분면 중 어디로 향하는지 나타냄?
	double mean_xy[2];
	mean_xy[0] = vx > 0 ? 1: -1;
	mean_xy[1] = vy > 0 ? 1: -1;

	// dx, dy 중어느 것이 더 큰지 검사.
	int x_or_y = 0;
	if(vx * vx < vy * vy)
		x_or_y = 1;

	double min = DBL_MAX / 2 * -1;
	double max = DBL_MAX / 2;

	CP_BSPNode *node = T;
	CP_BSPNode *child = NULL;

	while(node->parent != NULL){
		// root로 거슬러 올라가면서 partition과 교차하는지 검사
		child = node;
		node = node->parent;

		CP_Partition* t_bp = new CP_Partition(); // (tree)_(binary)(partition)
		if(child == node->leftChild){ 
			// 만약 현재 노드가 parent 기준 양의 영역에 있는 경우..
			t_bp->begin = node->partition->begin;
			t_bp->end = node->partition->end;
		}
		else{
			// 만약 현재 노드가 parent 기준 음의 영역에 있는 경우..
			// Q : 왜 바꿔주지? -> vector의 방향을 바꾸어서 내/외부 검사?
			t_bp->begin = node->partition->end;
			t_bp->end = node->partition->begin;
		}
		// (almost wrong) CP_Partition* t_bp = node->partition;
		CP_Vec2 t_vec, p_vec;
		CP_Point point = t_bp->intersection(partition, t_vec, p_vec);		

		// check if two vectors (t, p) are 'parallel'(cross product is zero)
		double cross_product_tp = t_vec.cross_product(p_vec);
		if(compare_float(cross_product_tp, 0)){
			//Now it is assumed that coincidence or parallel can be on the left side of T node-partition
			// 두 개의 시작점을 잇는 벡터..
			CP_Vec2 v = partition->begin - t_bp->begin;
			if(t_vec.cross_product(v) >= 0) {
				// 'v' is counterclockwise to the 'tb' or coincidence (inside or on)
				continue;
			}
			else{ 
				// partition이 T 바깥에 있는 것이 확실하므로 더이상 진행할 필요가 없음.
				return false;
			}
		}
		// 만약 두 개의 벡터가 평행하지 않은 경우...
		if(cross_product_tp > TOLERENCE)
		{
			double currentMin = (x_or_y == 0) ?
				(point.m_x - partition->begin.m_x) * mean_xy[x_or_y] : // == 0
				(point.m_y - partition->begin.m_y) * mean_xy[x_or_y];  // == 1

			if (currentMin >= max) return false;
			else
				if (currentMin > min) {
					min = currentMin;
					begin = point;
				}
		}
		else{
			double currentMax = (x_or_y == 0) ?
				(point.m_x - partition->begin.m_x) * mean_xy[x_or_y] : // == 0
				(point.m_y - partition->begin.m_y) * mean_xy[x_or_y];  // == 1

			if (currentMax <= min) return false;
			else
				if (currentMax < max) {
					max = currentMax;
					end = point;
				}
		}
	}

	if(x_or_y == 0)
		pcross = (cross.m_x - partition->begin.m_x) * mean_xy[x_or_y];
	else if(x_or_y == 1)
		pcross = (cross.m_y - partition->begin.m_y) * mean_xy[x_or_y];
	pmin = min;
	pmax = max;
	return true;	
}

/*
bool gb_t_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &pos, CP_Point *cross, 
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
	CP_Point point;
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
bool gb_isCross(CP_BSPNode* A, CP_Point &point){
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

void gb_complement(CP_BSPNode* T){
	if(gb_treeIsCell(T)){
		T->position = 3 - T->position;
		return;
	}
	gb_complement(T->leftChild);
	gb_complement(T->rightChild);
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
	if(T->position == 0){
		fout<<str<<"("<<T->partition->begin.m_x<<","<<T->partition->begin.m_y<<")---->("<<T->partition->end.m_x<<","<<T->partition->end.m_y<<")"<<endl;
		_debugFoutBsptree(T->leftChild, floor + 3, fout);
		_debugFoutBsptree(T->rightChild, floor + 3, fout);
	}
	else{	
		if(T->position == 1)
			fout<<str<<"IN"<<endl;
		else
			fout<<str<<"OUT"<<endl;
	}
	delete[] str;
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

		for(unsigned int i = 1; i < node->pos_coincident.size(); i++){//凜槨0角션쩌돨殮窟，痰黨털뙤T,P貫零珂션쩌T돨partition瞳혐堵코돨꼬롸
			p->begin = node->pos_coincident[i]->begin;
			p->end = node->pos_coincident[i]->end;

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
				CP_Partition *node_face = new CP_Partition();

				node_face->begin = p->begin;
				node_face->end = p->end;

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
			p->begin = node->neg_coincident[i]->begin;
			p->end = node->neg_coincident[i]->end;

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
				CP_Partition *node_face = new CP_Partition();

				node_face->begin = p->begin;
				node_face->end = p->end;

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
	while(node->parent != NULL){
		//generate polygon
		child = node;
		node = node->parent;
		CP_Partition *p = new CP_Partition();

		p->begin = node->partition->begin;
		p->end = node->partition->end;

		//Determine whether it contributes to the shape of the restricted cell polygon	
		CP_Partition *polygon_face = new CP_Partition();

		polygon_face->begin = node->partition->begin;
		polygon_face->end = node->partition->end;
		CP_Point begin;
		CP_Point end;
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
	else{ // p宅face殮窟貫零路북
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
				//혤놔f宅p路북돨꼬롸
				CP_Partition *result = new CP_Partition();
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
		result->begin = face->begin;
		result->end = face->end;
		return true;
	}
	else if(p_begin >= face_begin && p_end <= face_end){
		result->begin = p->begin;
		result->end = p->end;
		return true;
	}
	else if(p_begin <= face_begin && p_end > face_begin){
		result->begin = face->begin;
		result->end = p->end;
		return true;
	}
	else{
		result->begin = p->begin;
		result->end = face->end;
		return true;
	}
}

bool gb_p_in_cellPolygon(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point &end){
	begin = partition->begin;
	end = partition->end;

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

		if((-tb) * pa - (-pb) * ta >= -TOLERENCE && (-tb) * pa - (-pb) * ta <= TOLERENCE){//틱契 君瞳솝땍路북샀諒틱契瞳Tnode-partition璘긋떼옵鹿
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
						begin = point;
					}
			}
			else if(x_or_y == 1){
				double currentMin = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMin >= max)
					return false;
				else
					if(currentMin > min){
						min = currentMin;
						begin = point;
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
						end = point;
					}
			}
			else if(x_or_y == 1){
				double currentMax = (point.m_y - partition->begin.m_y) * mean_xy[x_or_y];
				if(currentMax <= min)
					return false;
				else
					if(currentMax < max){
						max = currentMax;
						end = point;
					}
			}
		}
	}

	return true;	
}