#ifndef TRIANGULATEPOLYGON_H
#define TRIANGULATEPOLYGON_H

#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

// Node types
#define T_X     1
#define T_Y     2
#define T_SINK  3

// Max number of segments that can be input
#define SEGSIZE 200

// Maximum table size
#define QSIZE   8*SEGSIZE
// Maximum number of trapezoids
#define TRSIZE  4*SEGSIZE

// Checking whether a point is inserted
#define FIRSTPT 1
#define LASTPT  2

// Infinity
#define INFINITY 100000000000
// Tolerance value
#define EPSILON 0.0000001

// For merge direction
#define S_LEFT  1
#define S_RIGHT 2

// For trapezium state
#define ST_VALID    1
#define ST_INVALID  2

// For splitting trapezoids
#define SP_SIMPLE_LRUP  1
#define SP_SIMPLE_LRDN  2
#define SP_2UP_2_DN     3
#define SP_2UP_LEFT     4
#define SP_2UP_RIGHT    5
#define SP_2DN_LEFT     6
#define SP_2DN_RIGHT    7
#define SP_NOSPLIT     -1

// For traverse direction
#define TR_FROM_UP  1
#define TR_FROM_DN  2

#define TRI_LHS 1
#define TRI_RHS 2

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#define CROSS(v0,v1,v2) (((v1).x - (v0).x)*((v2).y - (v0).y) - \
             ((v1).y - (v0).y)*((v2).x - (v0).x))

#define DOT(v0,v1) ((v0).x * (v1).x + (v0).y * (v1).y)

#define FP_EQUAL(s,t) (std::fabs(s-t) <= EPSILON)

struct point{
    double x;
    double y;
};

// Segment
struct segment{
    std::shared_ptr<point> v0, v1;      // two endpoints
    bool is_inserted;                   // inserted in trapezoidation yet?
    int root0, root1;                   // root nodes in Q
    int next;                           // next logical segment
    int prev;                           // previous segment
};

// Trapezoid
struct trapezoid{
    int lseg, rseg;                 // two adjoining segments
    std::shared_ptr<point> hi, lo;  // max/min y-values
    int u0, u1;
    int d0, d1;
    int sink;                       // pointer to cerrespondng in Q
    int usave, uside;               // ??
    int state;
};

// Node
struct node{
    int node_type;      // Y-Node or S-Node
    int seg_number;
    std::shared_ptr<point> yval;
    int trap_num;
    int parent;         // doubly linked DAG
    int left, right;    // children
};

struct monochain{
    int vnum;
    int next;           // circularly linked list
    int prev;           // describing the monotone
    int marked;         // polygon
};

struct vertexchain{
    std::shared_ptr<point> pt;
    Eigen::Vector4d vnext;      // next vertices for the 4 chains
    Eigen::Vector4d vpos;       // position of v the 4 chains
    int next_free;
};

int MonotonateTrapezoids(int);
int TriangulateMonotonePolygons(int,int,Eigen::Vector3d);

bool GreaterThan_(point v0, point v1);
bool EqualTo_(point v0, point v1);
bool GreaterThanEqualTo_(point v0, point v1);
bool LessThan_(point v0, point v1);

int LocateEndpoint(std::shared_ptr<point> v, std::shared_ptr<point> vo, int r);
int ConstructTrapezoids(int nseg);
int GenerateRandomOrdering(int);
int ChooseSegment();
int ReadSegments(std::string,std::vector<int>);
int MathLogstarN(int);
int MathN(int,int);

#endif // TRIANGULATEPOLYGON_H
