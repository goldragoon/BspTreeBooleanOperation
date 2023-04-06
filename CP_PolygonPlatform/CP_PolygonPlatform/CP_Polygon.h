#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

// for only debugging.
#include <iostream>

using namespace std;

// Math
#define DOUBLE_PI           6.28318530717958647692
#define PI                            3.14159265358979323846
#define HALF_PI                1.57079632679489661923
#define TOLERENCE 1e-4 // global tolerance

#define NOMINMAX 1

bool compare_float(double x, double y, double epsilon = 1e-4);

class CP_Vec2 {
public:
	double m_x, m_y;

	CP_Vec2() : m_x(0.0), m_y(0.0) {};
	CP_Vec2(const double _x, const double _y) : m_x(_x), m_y(_y) {};
	CP_Vec2(const CP_Vec2& v) : m_x(v.m_x), m_y(v.m_y) {};

	CP_Vec2& operator+ (const CP_Vec2& _rhs) {
		CP_Vec2 v;
		v.m_x = m_x + _rhs.m_x;
		v.m_y = m_y + _rhs.m_y;
		return v;
	}

	CP_Vec2& operator- (const CP_Vec2& _rhs) {
		CP_Vec2 v;
		v.m_x = m_x - _rhs.m_x;
		v.m_y = m_y - _rhs.m_y;
		return v;
	}
	
	double operator* (const CP_Vec2& _rhs) const {
		return m_x * _rhs.m_x + m_y * _rhs.m_y;
	}
	
	CP_Vec2& operator* (const double& _rhs) {
		CP_Vec2 v;
		v.m_x = m_x * _rhs;
		v.m_y = m_y * _rhs;
		return v;
	}

	CP_Vec2& operator/ (const double& _rhs) {
		CP_Vec2 v;
		v.m_x = m_x / _rhs;
		v.m_y = m_y / _rhs;
		return v;
	}

	CP_Vec2& operator/= (const double& _rhs) {
		m_x = m_x / _rhs;
		m_y = m_y / _rhs;
		return (*this);
	}



	void normalize() {
		double m = magnitude();
		m_x /= m;
		m_y /= m;
	}

	double magnitude() {
		return sqrt(magnitude_squared());
	}

	double magnitude_squared() {
		return m_x * m_x + m_y * m_y;
	}

	double cross_product(const CP_Vec2& cp) {
		return m_x * cp.m_y - m_y * cp.m_x;
	}
};

class CP_Vec3 {
public:
	double m_x, m_y, m_z;

	CP_Vec3() : m_x(0.0), m_y(0.0), m_z(0.0) {};
	CP_Vec3(const double _x, const double _y, const double _z) : m_x(_x), m_y(_y), m_z(_z) {};
	CP_Vec3(const CP_Vec3& v3) : m_x(v3.m_x), m_y(v3.m_y), m_z(v3.m_z) {};
	CP_Vec3(const CP_Vec2& v2, double _z) {
		m_x = v2.m_x;
		m_y = v2.m_y;
		m_z = _z;
	}

	CP_Vec3 cross_product(const CP_Vec3& cp) const {
		CP_Vec3 ret;
		ret.m_x = m_y * cp.m_z - m_z * cp.m_y;
		ret.m_y = m_z * cp.m_x - m_x * cp.m_z;
		ret.m_z = m_x * cp.m_y - cp.m_x * m_y;
		return ret;
	}
};

class CP_Point2 {
public:
	double m_x, m_y;

    CP_Point2() : m_x(0.0), m_y(0.0) {}
	CP_Point2(const double _x, const double _y) : m_x(_x), m_y(_y) {}
	CP_Point2(const CP_Vec2 &v) : m_x(v.m_x), m_y(v.m_y) {}
	~CP_Point2(){}

	CP_Vec2& operator- (const CP_Point2& _rhs) const {
		CP_Vec2 pt;
		pt.m_x = m_x - _rhs.m_x;
		pt.m_y = m_y - _rhs.m_y;
		return pt;
	}

	CP_Vec2& operator+ (const CP_Point2& _rhs) const {
		CP_Vec2 pt;
		pt.m_x = m_x + _rhs.m_x;
		pt.m_y = m_y + _rhs.m_y;
		return pt;
	}

	CP_Point2& operator= (const CP_Point2& _rhs){
		this->m_x = _rhs.m_x;
		this->m_y = _rhs.m_y;
		return *this;
	}

	double dist(const CP_Point2& _rhs) const {
		return (*this - _rhs).magnitude();
	}

	CP_Vec2 as_vec() const { return CP_Vec2(m_x, m_y); }
	//static bool equal(const CP_Point2& _rhs, double _tol = 1e-6) {}
};
typedef vector<CP_Point2> VT_PointArray;

class CP_Line2 {
public:
	// 2D line coefficients (a)x + (b)y + (c) = 0
	double a, b, c;
	CP_Line2() :a(0.0), b(0.0), c(0.0) {}
	CP_Line2(const CP_Vec2& _s, const CP_Vec2& _e) {
		// construct line equation by converting _s, _e as homogeneous coordinate.
		CP_Vec3 S(_s, 1), E(_e, 1);
		CP_Vec3 coeffs = S.cross_product(E);
		a = coeffs.m_x; b = coeffs.m_y; c = coeffs.m_z;
		double max_coeff = 2;//(std::max)({ a, b, c }); normalization 때문에 너무 계수가 너무 작아지면 망할 수 있음.
		a /= max_coeff; b /= max_coeff; c /= max_coeff;
	}

	CP_Line2(const CP_Point2& _s, const CP_Point2& _e) {
		// construct line equation by converting _s, _e as homogeneous coordinate.
		CP_Vec3 S(_s.as_vec(), 1), E(_e.as_vec(), 1);
		CP_Vec3 coeffs = S.cross_product(E);
		a = coeffs.m_x; b = coeffs.m_y; c = coeffs.m_z;
		//double max_coeff = (std::max)({ std::abs(a), std::abs(b), std::abs(c) });
		//a /= max_coeff; b /= max_coeff; c /= max_coeff;
	}

	CP_Line2(const CP_Vec3& _vec3) : a(_vec3.m_x), b(_vec3.m_y), c(_vec3.m_z) {}

	bool isParallel(const CP_Line2& _line) const {
		return compare_float(a * _line.b - b * _line.a, 0);
	}

	CP_Vec3 as_vec() const {
		return CP_Vec3(a, b, c);
	}
	
	long double eval(const CP_Point2& _pt) {
		long double term1 = a * _pt.m_x;
		long double term2 = b * _pt.m_y;
		long double term3 = c;
		return (long double)(term1 + term2 + term3);
	}

	//bool isCoincide(const CP_Line2& _line) const {}
};

class CP_Polygon;

typedef vector<int> VT_IntArray;
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
};
typedef vector<CP_Region> VT_RegionArray;

class CP_Polygon
{
public:
    VT_PointArray m_pointArray;
    VT_RegionArray m_regionArray;

public:
    void mb_clear( ) {
		m_pointArray.clear( ); 
		m_regionArray.clear( );
	}
};

extern void     gb_distanceMinPointLoop(double&d, int& idRegion, int& idLoop, CP_Point2& pt, CP_Polygon& pn);
extern void     gb_distanceMinPointPolygon(double&d, int& id, CP_Point2& pt, CP_Polygon& pn);
extern double   gb_distancePointSegment(CP_Point2& pt, CP_Point2& p1, CP_Point2& p2);

extern bool     gb_findPointInLoop(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, int pointInPolygon);
extern void     gb_insertPointInPolygon(CP_Polygon& pn, int& idRegion, int& idLoop, int& idPointInLoop, CP_Point2& newPoint);

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

extern void     gb_pointConvertFromGlobalToScreen(CP_Point2& result, CP_Point2 pointGlobal, double scale, CP_Point2 translation, int screenX, int screenY);
extern void     gb_pointConvertFromScreenToGlobal(CP_Point2& result, CP_Point2 pointScreen, double scale, CP_Point2 translation, int screenX, int screenY);
extern bool     gb_polygonNewInLoopRegular(CP_Polygon& p, int idRegion, int n, double r, double cx, double cy);
extern void     gb_polygonNewOutLoopRegular(CP_Polygon& p, int n, double r, double cx, double cy);
extern bool     gb_removeLoop(CP_Polygon& pn, int idRegion, int idLoop);
extern bool     gb_removePoint(CP_Polygon& pn, int id);
extern bool     gb_removeRegion(CP_Polygon& pn, int idRegion);
extern void     gb_subtractOneAboveID(CP_Polygon& pn, int id);
extern bool     gb_checkPolygon(CP_Polygon& pn);
extern bool     gb_checkRegion(CP_Region& rn);
extern bool     gb_checkRegionCrossRegion(CP_Region &region1, CP_Region &region2);
extern bool     gb_checkLoopSelfIntersection(CP_Loop& ln);
extern bool     gb_checkLoopIntersection(CP_Loop& lnin1, CP_Loop& lnin2);
extern bool     gb_checkLineSegmentCross(CP_Point2* a1, CP_Point2* a2, CP_Point2* b1, CP_Point2* b2);

class CP_BSPNode;
extern CP_BSPNode* gb_buildPolygonBSPTree(CP_Polygon& pn);
extern CP_BSPNode* gb_buildRegionBSPTree(CP_Region& rn);
extern CP_BSPNode* gb_buildLoopBSPTree(CP_Loop& ln);

class CP_Partition{
public:
	CP_Point2 begin, end;

	// 어떤 point의 CP_Partition에 대한 상대적 위치를 나타냄.
	enum class PointSideness {
		LINE_IN, LINE_POS, LINE_NEG,
	};

	CP_Partition() {}
	CP_Partition(const CP_Point2 &b, const CP_Point2 &e) : begin(b), end(e) {}
	CP_Partition(const CP_Partition &p) { begin = p.begin; end = p.end; }
	~CP_Partition() {}

	double slope() {
		CP_Line2 t_line(this->begin.as_vec(), this->end.as_vec());
		return -(t_line.a / t_line.b);
	}

	/*
	* \brief 벡터(Partition = end - begin)를 무한한 직선으로 생각하여, 두 개의 직선 간의 intersection point를 구한다.
	*/
	CP_Point2 intersection(const CP_Partition& _partition, 
		CP_Vec2 &_t_vec, CP_Vec2 &_p_vec,
		CP_Line2 &_t_line, CP_Line2 &_p_line
	) const {
		// Proof : https://imois.in/posts/line-intersections-with-cross-products/

		CP_Point2 point_intersection;

		// calculate line equation coefficients 
		// 1. (this) object
		CP_Vec2 t_vec = this->end - this->begin;
		CP_Line2 t_line(this->begin.as_vec(), this->end.as_vec());
		CP_Vec3 t_line_coeff = t_line.as_vec();
		// 2. (_partition) object
		CP_Vec2 p_vec = _partition.end - _partition.begin;
		CP_Line2 p_line(_partition.begin.as_vec(), _partition.end.as_vec());
		CP_Vec3 p_line_coeff = p_line.as_vec();

		CP_Vec3 cp = t_line_coeff.cross_product(p_line_coeff);
		// projective/homogeneous equation으로 교점 구하기 (3차원 버전에서는 교선이 될 것임)
		point_intersection.m_x = cp.m_x / cp.m_z;
		point_intersection.m_y = cp.m_y / cp.m_z;

		// additional return for performance optimization
		_t_vec = t_vec; _p_vec = p_vec;
		_t_line = t_line; _p_line = p_line;

		return point_intersection;
	}

	/*
	* \brief 파티션(direction = end - begin)를 무한한 길이를 가지는 벡터로 생각하고
	* point가 벡터로 나눠지는 공간의 (벡터의 진행 방향) 왼쪽에 있으면 true, (벡터의 진행 방향) 오른쪽에 있으면 false를 반환한다.
	*/
	bool is_left_side(const CP_Point2& point) const{ // is_ccw_side 가 더 정확하지 않을까..
		CP_Vec2 partition_vec = end - begin;
		// cross product의 floating-point overflow를 막기위한 normalization이 필요할 수 있음.
		//partition_vec.normalize(); 

		// end, begin 중 아무거나 골라도 상관없음..
		CP_Vec2 partition2point = point - end;
		// cross product의 floating-point overflow를 막기위한 normalization이 필요할 수 있음.
		//partition2point.normalize();

		if (partition_vec.cross_product(partition2point) > 0) return true; // left (inside)
		else return false; // on or right (outside)
	}

	/*
	* \brief 파티션(direction = end - begin)를 무한한 길이를 가지는 벡터로 생각하고
	* 다른 벡터 _p가 this 파티션을 기준으로 CCW(left)로 회전하면 true 아니면 false를 반환한다.
	*/
	bool is_ccw_rot(const CP_Vec2 &_v) const {
		CP_Vec2 partition_vec = end - begin;
		if (partition_vec.cross_product(_v) > 0) return true; // left (inside)
		else return false; // on or right (outside)
	}

	/*
	* \brief check that point(_pt) is on is partition line segment(end ~ begin).
	*/
	bool is_point_on_lineseg(const CP_Point2 &_pt) const {
		double d;
		closestPoint(_pt, d);
		return compare_float(d, 0);
	}

	/*
	* \brief check that point equivalent is partition line segment points(end, begin).
	*/
	bool is_point_on_points(const CP_Point2& _pt) const {
		return compare_float(_pt.dist(this->begin), 0) && compare_float(_pt.dist(this->end), 0);
	}

	bool is_point_on_line(const CP_Point2& _pt) const {
		return compare_float(_pt.dist(this->begin), 0) && compare_float(_pt.dist(this->end), 0);
	}

	CP_Point2 closestPoint(const CP_Point2& point, double& d) const
	{
		CP_Vec2 dir = end - begin;
		d = std::clamp(((point - begin) * dir) / dir.magnitude_squared(), 0.0, 1.0);
		return begin + dir * d;
	}

	// 목적이 불분명.
	/*
	* gb_t_p_Position3에서 P_T_BOTH_POS, P_T_BOTH_NEG를 반환하고/검사하는 용도로만 사용됨.
	*/
	PointSideness coincidentPos(const CP_Point2& point) const {
		CP_Vec2 begin2point = point - begin; // vector
		CP_Vec2 end2point = point - end;     // vector

		// point가 partition line 위에 있지 않다면...? 근데 왜 전부 or case로 검사해야되지...
		CP_Line2 _p(begin, end);
		double eval = _p.eval(point);
		
		if (
			(begin2point.m_x > TOLERENCE && end2point.m_x < -TOLERENCE)
			|| (begin2point.m_x < -TOLERENCE && end2point.m_x > TOLERENCE)
			|| (begin2point.m_y > TOLERENCE && end2point.m_y < -TOLERENCE)
			|| (begin2point.m_y < -TOLERENCE && end2point.m_y > TOLERENCE))
		{
			if (eval > TOLERENCE) {
				//printf("coincidentPos eval is bigger than 0\n");
			}

			if (eval < -TOLERENCE) {
				//printf("coincidentPos eval is smaller than 0\n");
			}
			return PointSideness::LINE_IN;
		}
		else {

			if (compare_float(eval, 0)) {
				//printf("WHY? : coincidentPos eval is 0\n");
			}

			double dx1, dy1, dx2, dy2;
			dx1 = std::abs(begin2point.m_x);
			dy1 = std::abs(begin2point.m_y);

			dx2 = std::abs(end2point.m_x);
			dy2 = std::abs(end2point.m_y);

			if (dx1 + dy1 < dx2 + dy2) {
				return PointSideness::LINE_NEG;
			}
			else {
				return PointSideness::LINE_POS;
			}
		}
	}

	CP_Partition& operator= (const CP_Partition& p) {
		if (this == &p)
			return *this;

		this->begin = p.begin;
		this->end = p.end;
		return *this;
	}
};

enum class CP_BSPOp {
	UNION,
	INTERSECTION,
	SUBTRACTION
};

class CP_BSPNode{
public:
	// relationship with 'other bsp nodes'.
	CP_BSPNode *parent;
	CP_BSPNode *leftChild;
	CP_BSPNode *rightChild;
	
	// 현재 노드의 binary partition과, 나뉘어진 하위 파티션들..
	CP_Partition partition;
	vector<CP_Partition> pos_coincident;
	vector<CP_Partition> neg_coincident;

	// visualization/output purpose.
	vector<CP_Partition*> polygon;
	vector<CP_Partition*> leftIn;
	vector<CP_Partition*> leftOut;
	vector<CP_Partition*> rightIn;
	vector<CP_Partition*> rightOut;

	/*
	* \brief BSP Node의 BSPTree 기준 위치를 나타냄.
	* \details UNDEFINED의 존재 이유?
	*/ 
	enum class Sideness {
		UNDEFINED, INSIDE, OUTSIDE
	};

	Sideness side; // 0 : Nothing , 1 : Region In, 2 : Region Out 

	CP_BSPNode() : parent(NULL), leftChild(NULL), rightChild(NULL), partition(), side(Sideness::UNDEFINED){}
	CP_BSPNode(const CP_BSPNode* const node){
		parent = node->parent;
		leftChild = node->leftChild;
		rightChild = node->rightChild;
		partition = node->partition;
		side = node->side;
		for (auto& pc : node->pos_coincident)
			pos_coincident.push_back(pc);

		for (auto& nc : node->neg_coincident)
			neg_coincident.push_back(nc);
	}
	void copy(const CP_BSPNode* const node){
		parent = node->parent;
		leftChild = node->leftChild;
		rightChild = node->rightChild;
		partition = node->partition;
		side = node->side;
		for (auto& pc : node->pos_coincident)
			pos_coincident.push_back(pc);

		for (auto& nc : node->neg_coincident)
			neg_coincident.push_back(nc);
	}

	void assign_coincidents(const CP_BSPNode* const node) {
		for (auto& pc : node->pos_coincident)
			pos_coincident.push_back(pc);

		for (auto& nc : node->neg_coincident)
			neg_coincident.push_back(nc);
	}

	bool isCell() const {
		return (leftChild == NULL) && (rightChild == NULL);
	}

	void complement() {_complement(this);}
	void _complement(CP_BSPNode* node) {
		if (node->isCell()) {
			node->side = (node->side == Sideness::INSIDE) ? Sideness::OUTSIDE : Sideness::INSIDE;
			return;
		}
		_complement(node->leftChild);
		_complement(node->rightChild);
	}
	~CP_BSPNode(){}
};

// [BSP Build Merge Both]
// - partition이 T의 내부에 있는지 검사한다.
extern bool gb_p_in_region(
	const CP_BSPNode* const T, const CP_Partition& partition, // input parameters
	CP_Partition& partition_spl // chunk of input 'partition' that is splited by the region of 'T'.
);
//extern bool gb_t_in_region(CP_BSPNode* T, CP_Partition* partition, CP_Point2 &pos, CP_Point2 *cross, double &pmin, double &pmax, double &pcross);

// [BSP Build Related]
extern CP_BSPNode* gb_buildBSPTree(const vector<CP_Partition> &vp, CP_BSPNode* parent, char childInfo);
#define CHILDINFO_NO 0
#define CHILDINFO_LEFT 1
#define CHILDINFO_RIGHT 2
extern void gb_getCrossPartition(const CP_Partition& T, const CP_Partition& P, CP_Partition &left, CP_Partition &right);
extern char getPartitionPos(const CP_Partition& partition, const CP_Partition& H);

// [BSP Merge Related]
extern CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, CP_BSPNode* parent, CP_BSPOp op, bool left);
extern CP_BSPNode* gb_mergeBSPTree(CP_BSPNode* A, CP_BSPNode* B, CP_BSPOp op);
extern CP_BSPNode* gb_mergeTreeWithCell(CP_BSPNode* T1, CP_BSPNode* T2, CP_BSPOp op);
extern void gb_partitionBspt(
	const CP_BSPNode* const T, const CP_Partition& partition, 
	CP_BSPNode* &B_inLeft, CP_BSPNode* &B_inRight, CP_BSPNode* parent, 
	const CP_Partition& splited_partition);

// The original method of judging the positional relationship between T and P when dividing Bsptree takes O(n^ 2)
//extern char gb_t_p_Position(CP_BSPNode* A, CP_Partition* partition, CP_Point2& point, CP_Point2& partitionLBegin, CP_Point2& partitionLEnd, CP_Point2& partitionRBegin, CP_Point2& partitionREnd);

// New improved method for judging the positional relationship between T and P when splitting Bsptree,
// time-consuming is O(n)
extern char gb_t_p_Position3(const CP_BSPNode* const A, const CP_Partition &partition, 
	CP_Point2& cross_point, 
	CP_Partition& partitionL, CP_Partition& partitionR);

// [Visualization & Output Related]
// Lazy TODO : detach CP_Partition pointer from here. (not crucial.. these are just visualization methods.)
extern bool gb_generateCellPolygon(CP_BSPNode *cell);
extern bool gb_generateCellPolygonPre(CP_BSPNode *cell);
extern bool gb_generateCellPolygons(CP_BSPNode *root);
extern bool gb_changePartitionDir(CP_Partition *p);
extern bool gb_p_in_cellPolygon(CP_BSPNode* T, CP_Partition* partition, CP_Point2 &begin, CP_Point2 &end);
extern bool gb_generateBSPTreeFaces(CP_BSPNode *root);
extern bool gb_generateBSPTreeFace(CP_BSPNode *node);
extern bool gb_cutParallelFace(CP_Partition *p, CP_Partition *face, CP_Partition *result);
extern bool gb_cutPolygonFace(CP_Partition *p, CP_Partition *face);
extern bool gb_treeHasInCell(CP_BSPNode* tree);
extern bool gb_tree1OverlapWithTree2(CP_BSPNode* tree1, CP_BSPNode* tree2);
extern bool gb_tree1InTree2(CP_BSPNode* tree1, CP_BSPNode* tree2);

// [Pure Debugging Purpose]
extern void debugBsptree(CP_BSPNode* T);
extern void _debugFoutBsptree(CP_BSPNode* T, int floor, ofstream& fout);// call by debugBsptree

// [MFC Memory Management.]
extern void releaseMemory();

// Classification of bp(binary partitioner) to other bp.
#define POS_LEFT 0
#define POS_POS_ON 1
#define POS_NEG_ON 2
#define POS_RIGHT 3
#define POS_CROSS 4

// pair-ed classification of binary partitioner.
/*
* \brief this seven cases are defined by [Nayler et al, 1990] Chapter 3, (check figure 3.1)
* \details left side of partition is pos, right side of partition is neg
*/
#define P_T_ON_POS 0 // Anti-parallel On
#define P_T_ON_NEG 1 // Parallel On
#define P_T_POS_NEG 2 //
#define P_T_POS_POS 3
#define P_T_NEG_POS 4
#define P_T_NEG_NEG 5
#define P_T_BOTH_POS 6
#define P_T_BOTH_NEG 7