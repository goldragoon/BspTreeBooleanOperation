#ifndef CP_POLYGON_H
#define CP_POLYGON_H

#include <iostream>
#include <vector>
using namespace std;

#define DOUBLE_PI           6.28318530717958647692
#define PI                            3.14159265358979323846
#define HALF_PI                1.57079632679489661923

class CP_Point
{
public:
    double m_x, m_y;
public:
    CP_Point(void):m_x(0.0),m_y(0.0){}
	~CP_Point(){}
	CP_Point& operator= (const CP_Point& _rhs){
		this->m_x = _rhs.m_x;
		this->m_y = _rhs.m_y;
		return *this;
	}
};

typedef vector<CP_Point> VT_PointArray;

class CP_Polygon;

typedef vector<int> VT_IntArray;
typedef vector<VT_IntArray> VT_IntArray2;

class CP_Loop
{
public:
    VT_IntArray m_pointIDArray;
    int m_loopIDinRegion;
    int m_regionIDinPolygon;
    CP_Polygon* m_polygon;

public:
    CP_Loop(void):m_loopIDinRegion(0),m_regionIDinPolygon(0),m_polygon(NULL) { }
};
typedef vector<CP_Loop> VT_LoopArray;

class CP_Region
{
public:
    VT_LoopArray m_loopArray;
    int m_regionIDinPolygon;
    CP_Polygon* m_polygon;

public:
    CP_Region(void):m_regionIDinPolygon(0),m_polygon(NULL) { }
}; // 类CP_Region定义结束
typedef vector<CP_Region> VT_RegionArray;

class CP_Polygon
{
public:
    VT_PointArray m_pointArray;
    VT_RegionArray m_regionArray;

public:
    void mb_clear( ) {m_pointArray.clear( ); m_regionArray.clear( );}
}; // 类CP_Polygon定义结束

extern void     gb_distanceMinPointLoop(double&d, int& idRegion, int& idLoop,
                             CP_Point& pt, CP_Polygon& pn);
extern void     gb_distanceMinPointPolygon(double&d, int& id, CP_Point& pt, CP_Polygon& pn);
extern double   gb_distancePointPoint(CP_Point& p1, CP_Point& p2);
extern double   gb_distancePointSegment(CP_Point& pt, CP_Point& p1, CP_Point& p2);

extern void     gb_getIntArrayPointInPolygon(VT_IntArray& vi, CP_Polygon& pn, CP_Point& p, double eT);
extern bool     gb_findPointInLoop(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, int pointInPolygon);
extern void     gb_insertPointInPolygon(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, CP_Point& newPoint);

extern void     gb_intArrayInit(VT_IntArray& vi, int data);
extern void     gb_intArrayInitLoop(VT_IntArray& vi, CP_Polygon& pn, int idRgion, int idLoop, double eT);
extern void     gb_intArrayInitPoint(VT_IntArray& vi, CP_Polygon& pn, int v, double eT);
extern void     gb_intArrayInitPointSame(VT_IntArray& vi, CP_Polygon& pn, double eT);
extern void     gb_intArrayInitPolygon(VT_IntArray& vi, CP_Polygon& pn);
extern void     gb_intArrayInitPolygonSamePoint(VT_IntArray& vr, CP_Polygon& pr, VT_IntArray& vs, CP_Polygon& ps, double eT);
extern void     gb_intArrayInitRegion(VT_IntArray& vi, CP_Polygon& pn, int idRegion, double eT);

extern void     gb_moveLoop(CP_Polygon& pn, int idRegion, int idLoop, double vx, double vy);
extern void     gb_movePoint(CP_Polygon& pn, int id, double vx, double vy);
extern void     gb_movePointIntArray(CP_Polygon& pn, VT_IntArray& vi, double vx, double vy);
extern void     gb_movePolygon(CP_Polygon& pn, double vx, double vy);
extern void     gb_moveRegion(CP_Polygon& pn, int idRegion, double vx, double vy);

extern void     gb_pointConvertFromGlobalToScreen(CP_Point& result, CP_Point pointGlobal, double scale, CP_Point translation, int screenX, int screenY);
extern void     gb_pointConvertFromScreenToGlobal(CP_Point& result, CP_Point pointScreen, double scale, CP_Point translation, int screenX, int screenY);
extern bool     gb_polygonNewInLoopRegular(CP_Polygon& p, int idRegion, int n, double r, double cx, double cy);
extern void     gb_polygonNewOutLoopRegular(CP_Polygon& p, int n, double r, double cx, double cy);
extern bool     gb_removeLoop(CP_Polygon& pn, int idRegion, int idLoop);
extern bool     gb_removePoint(CP_Polygon& pn, int id);
extern bool     gb_removeRegion(CP_Polygon& pn, int idRegion);
extern void     gb_subtractOneAboveID(CP_Polygon& pn, int id);


//my code
//check
class CP_BSPNode;
extern bool     gb_checkPolygon(CP_Polygon& pn);
extern bool     gb_checkRegion(CP_Region& rn);
extern bool     gb_checkReginInRegin(CP_Region &in, CP_Region &region);
extern bool     gb_checkRegionCrossRegion(CP_Region &region1, CP_Region &region2);
extern bool     gb_checkLoopSelfIntersection(CP_Loop& ln);
extern bool     gb_checkLoopIntersection(CP_Loop& lnin1, CP_Loop& lnin2);
extern bool     gb_checkLineSegmentCross(CP_Point* a1, CP_Point* a2, CP_Point* b1, CP_Point* b2);
extern CP_BSPNode* gb_buildPolygonBSPTree(CP_Polygon& pn);
extern CP_BSPNode* gb_buildRegionBSPTree(CP_Region& rn);
extern CP_BSPNode* gb_buildLoopBSPTree(CP_Loop& ln);

//bsptree
class CP_Partition{
public:
	CP_Point begin;
	CP_Point end;
    vector<CP_Point> point_list;
	vector<CP_Partition*> partition_list;

	CP_Partition() {
		begin = CP_Point();
		end = CP_Point();
		point_list.push_back(begin);
		point_list.push_back(end);
	}

	CP_Partition(CP_Point &b, CP_Point &e){
		begin = CP_Point();
		end = CP_Point();
		begin = b;
		end = e;
		point_list.push_back(begin);
		point_list.push_back(end);
	}

	~CP_Partition(){
		for(unsigned int i = 0; i < partition_list.size(); i++)
			delete partition_list[i];
	}

	CP_Partition* operator= (CP_Partition* p){
		CP_Partition* r = new CP_Partition();
		r->begin = p->begin;
		r->end = p->end;
		partition_list.push_back(r);
		return r;
	}
};

typedef vector<CP_Partition> VT_PartitionArray;
extern void     gb_getLoopPartition(CP_Loop& ln, VT_PartitionArray& vp);


class CP_BSPNode{
public:
	CP_BSPNode *parent;
	CP_BSPNode *leftChild;
	CP_BSPNode *rightChild;
	CP_Partition * partition;
	vector<CP_Partition*> pos_coincident;
	vector<CP_Partition*> neg_coincident;
	vector<CP_Partition*> polygon;//cell 记录 polygon，node记录face

	vector<CP_Partition*> leftIn;
	vector<CP_Partition*> leftOut;
	vector<CP_Partition*> rightIn;
	vector<CP_Partition*> rightOut;
	char position;

	CP_BSPNode():parent(NULL), leftChild(NULL), rightChild(NULL), partition(NULL), position(0){}
	CP_BSPNode(CP_BSPNode *node){
		parent = node->parent;
		leftChild = node->leftChild;
		rightChild = node->rightChild;
		partition = node->partition;
		position = node->position;
		for(unsigned int i = 0; i < node->pos_coincident.size(); i++){
			pos_coincident.push_back(node->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < node->neg_coincident.size(); i++){
			neg_coincident.push_back(node->neg_coincident[i]);
		}
	}
	void copy(CP_BSPNode *node){
		parent = node->parent;
		leftChild = node->leftChild;
		rightChild = node->rightChild;
		partition = node->partition;
		position = node->position;
		for(unsigned int i = 0; i < node->pos_coincident.size(); i++){
			pos_coincident.push_back(node->pos_coincident[i]);
		}
		for(unsigned int i = 0; i < node->neg_coincident.size(); i++){
			neg_coincident.push_back(node->neg_coincident[i]);
		}
	}
	~CP_BSPNode(){}
};

extern bool     gb_treeHasInCell(CP_BSPNode* tree);
extern bool     gb_tree1OverlapWithTree2(CP_BSPNode* tree1, CP_BSPNode* tree2);
extern bool     gb_tree1InTree2(CP_BSPNode* tree1, CP_BSPNode* tree2);


extern CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, CP_BSPNode* parent, char op, bool left);
extern CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, char op);
extern CP_BSPNode* gb_buildBSPTree(vector<CP_Partition*> &vp, CP_BSPNode* parent, char childInfo);
#define CHILDINFO_NO 0
#define CHILDINFO_LEFT 1
#define CHILDINFO_RIGHT 2
extern void gb_getCrossPartition(CP_Partition* T, CP_Partition* P, CP_Partition* &left, CP_Partition* &right);
extern char getPatitionPos(vector<CP_Partition*> &vp, int pos, CP_Partition *H);
extern CP_BSPNode* gb_mergeTreeWithCell(CP_BSPNode* T1, CP_BSPNode* T2, char op);
extern bool gb_treeIsCell(CP_BSPNode* node);
extern void gb_partitionBspt(CP_BSPNode* T, CP_Partition* partition, CP_BSPNode* &B_inLeft, CP_BSPNode* &B_inRight, CP_BSPNode* root, CP_Point& partitionBegin, CP_Point& partitionEnd);

// The original method of judging the positional relationship between T and P when dividing Bsptree takes O(n^ 2)
extern char gb_t_p_Position(CP_BSPNode* A, CP_Partition* partition, CP_Point& point, CP_Point& partitionLBegin, CP_Point& partitionLEnd, CP_Point& partitionRBegin, CP_Point& partitionREnd);

// New improved method for judging the positional relationship between T and P when splitting Bsptree, time-consuming is O(n)
extern char gb_t_p_Position3(CP_BSPNode* A, CP_Partition* partition, CP_Point& point, CP_Point& partitionLBegin, CP_Point& partitionLEnd, CP_Point& partitionRBegin, CP_Point& partitionREnd);

extern bool gb_isCross(CP_BSPNode* A, CP_Point &point);
extern void gb_complement(CP_BSPNode* T);
extern char gb_coincidentPos(CP_Partition *p, CP_Point &point);
extern bool gb_p_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point &end, CP_Point *cross, double &pmin, double &pmax, double &pcross);

extern bool gb_t_p_left(CP_Partition* tp, CP_Partition* partition);
extern bool gb_parent_t_sameDirection(CP_Partition *p1, CP_Partition *p2);
extern bool gb_point_partition_near_begin(CP_Partition* partition, CP_Point *point);
extern bool gb_generateCellPolygon(CP_BSPNode *cell);
extern bool gb_generateCellPolygonPre(CP_BSPNode *cell);
extern bool gb_generateCellPolygons(CP_BSPNode *root);
extern bool gb_changePartitionDir(CP_Partition *p);
extern bool gb_p_in_cellPolygon(CP_BSPNode* T, CP_Partition* partition, CP_Point &begin, CP_Point &end);
extern bool gb_generateBSPTreeFaces(CP_BSPNode *root);
extern bool gb_generateBSPTreeFace(CP_BSPNode *node);

extern bool gb_t_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point &pos, CP_Point *cross, double &pmin, double &pmax, double &pcross);
extern bool gb_t_p_left(CP_Point &point, CP_Partition* partition);
//extern bool gb_generateBSP(CP_BSPNode *node);
extern bool gb_cutParallelFace(CP_Partition *p, CP_Partition *face, CP_Partition *result);
extern bool gb_cutPolygonFace(CP_Partition *p, CP_Partition *face);
extern void debugBsptree(CP_BSPNode* T);
extern void debugFoutBsptree(CP_BSPNode* T, int floor);

extern void releaseMemory();

/////////////////////////////////////////////////////////////////////////////////////////////////////////没有使用，此函数用于将分割p修剪到只剩下属于T区域中的线段，有bug暂时没用
extern bool gb_cutPByRegionOfT(CP_BSPNode* T, CP_Partition* partition, CP_Point &pBegin, CP_Point& pEnd);
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define OP_UNION 0
#define OP_INTERSECTION 1
#define OP_DIFFERENCE 2

#define POS_LEFT 0
#define POS_POS_ON 1
#define POS_NEG_ON 2
#define POS_RIGHT 3
#define POS_CROSS 4

#define REGION_IN 1
#define REGION_OUT 2
#define TOLERENCE 1e-4


//左边pos 右边neg
#define P_T_ON_POS 0
#define P_T_ON_NEG 1
#define P_T_POS_NEG 2
#define P_T_POS_POS 3
#define P_T_NEG_POS 4
#define P_T_NEG_NEG 5
#define P_T_BOTH_POS 6
#define P_T_BOTH_NEG 7

#define LINE_IN 0
#define LINE_POS 1
#define LINE_NEG 2
#endif

