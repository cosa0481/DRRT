/* kdtree.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <string>
#include <vector>
#include <math.h>
#include <DRRT/kdtreenode.h>
#include <DRRT/heap.h>
#include <DRRT/list.h>
#include <DRRT/jlist.h>
#include <DRRT/ghostPoint.h>
#include <DRRT/drrt_data_structures.h>
#include <DRRT/drrt_distance_functions.h>
#include <DRRT/edge.h>

// Infinity value for distance
#define INF 1000000000000

// A KD-Tree data structure that stores nodes of type T
class KDTree {
public:
    int d;                          // the number of dimensions in the space (5)
    std::string distanceFunction;   // distance function to use: f(poseA, poseB)
    int treeSize;                   // the number of nodes in the KD-Tree

    int numWraps;                   // the total number of dimensions that wrap
    Eigen::VectorXi wraps;          // a vector of length d containing a list of all the
                                    // dimensions that wrapAround
    Eigen::VectorXd wrapPoints;     // space is assumed to start at 0 and end at
                                    // wrapPoints[i] along dimension wraps[i]
    KDTreeNode* root;               // the root node

    // Constructors
    KDTree( int _d, std::string _distanceFunction,
            Eigen::VectorXi _wraps, Eigen::VectorXd _wrapPoints ) :
            d(_d), distanceFunction(_distanceFunction), treeSize(0),
            numWraps(_wraps.size()), wraps(_wraps), wrapPoints(_wrapPoints)
    {}
    KDTree( int _d, std::string _distanceFunction ) :
            d(_d), distanceFunction(_distanceFunction), treeSize(0), numWraps(0)
    {}
    KDTree() : d(0), distanceFunction("none"), treeSize(0), numWraps(0)
    {}
};

// Returns the distance between pointA and pointB using distanceFunction
double distFunc( std::string distanceFunction, Eigen::VectorXd pointA, Eigen::VectorXd pointB)
{
    if( distanceFunction == "R3Dist" ) {
        return sqrt((pointA[1]-pointB[1])*(pointA[1]-pointB[1])
                + (pointA[2]-pointB[2])*(pointA[2]-pointB[2]) ) +
                sqrt((pointA[3]-pointB[3])*(pointA[3]-pointB[3])
                + (pointA[4]-pointB[4])*(pointA[4]-pointB[4]) ) +
                sqrt((pointA[5]-pointB[5])*(pointA[5]-pointB[5])
                + (pointA[6]-pointB[6])*(pointA[6]-pointB[6]) );
    } else if( distanceFunction == "R3SDist" ) {
        return R3SDist(pointA,pointB);
    } else if( distanceFunction == "EuclideanDist" ) {
        return EuclideanDist(pointA,pointB);
    } else if( distanceFunction == "threeTwoDRobotDist" ) {
        // drrt.h >> distance(), Wdist()
        return EuclideanDist(pointA,pointB);
    }

    return 0.0;
}

void KDTreeInit( KDTree* K, int d, std::string distanceFunction );

// Inserts a new node into the tree
bool kdInsert( KDTree* tree, KDTreeNode* node );

/////////////////////// Nearest ///////////////////////

// Returns the nearest node to the queryPoint in the subtree starting
// at root and also its distance. It takes a suggestion for a possible
// closest node and uses that if it is best
bool kdFindNearestInSubtree( KDTreeNode* nearestNode,
                                double* nearestNodeDist,
                                std::string distanceFunction,
                                KDTreeNode* root,
                                Eigen::VectorXd queryPoint,
                                KDTreeNode* suggestedClosestNode,
                                double suggestedClosestDist );

// Returns the nearest node to the queryPoint and also its distance
bool kdFindNearest( KDTreeNode* nearestNode, double* nearestNodeDist,
                    KDTree* tree, Eigen::VectorXd queryPoint );

// Returns the nearest node to queryPoint in the subtree starting at
// the root and also its distance. It also takes a suggestion for a
// possible closest node (and uses that if it is best)
bool kdFindNearestinSubtreeWithGuess( KDTreeNode* nearestNode,
                                        double* nearestNodeDist,
                                        std::string distanceFunction,
                                        KDTreeNode* root,
                                        Eigen::VectorXd queryPoint,
                                        KDTreeNode* suggestedClosestNode,
                                        double suggestedClosestDist );

// Returns the nearest node to the queryPoint and also its distance
// Instead of starting at the root, starts at guess
bool kdFindNearestWithGuess( KDTreeNode* nearestNode, double* nearestNodeDist,
                              KDTree* tree, Eigen::VectorXd queryPoint, KDTreeNode* guess );

/////////////////////// K Nearest ///////////////////////

// Adds the node to the heap if there is space in the current heap
// without growing past k, otherwise the current top is removed
// and then the top is returned

KDTreeNode* addToKNNHeap( BinaryHeap* H, KDTreeNode* node, double key, int k );

/* Finds the K nearest nodes to queryPoint in the subtree starting
 * at root. Note that this return data is stored in the nearestHeap,
 * and the heap may also contain nodes before this function is called
 * explicitly. Returns the node of the nearest set that is FARTHEST
 * from the queryPoint along with its distance. Assumes that there
 * is at least one node in the heap to begin with. IF this is the
 * first call to this function (e.g. from kdFindKNearest) then it
 * is also assumed that a dummy node has been added that has an INF
 * key value. This makes things easier with checking that all K slots
 * are used during the recursion
 */
bool kdFindKNearestInSubtree( KDTreeNode* farthestNode, double* farthestNodeDist,
                              std::string distanceFunction, KDTreeNode* root, int k,
                              Eigen::VectorXd queryPoint, BinaryHeap* nearestHeap );

// Returns the K nearest nodes to queryPoint and olso their distances
// Note that they are not sorted but are in reverse heap order
std::vector<KDTreeNode> kdFindKNearest( KDTree* tree, int k, Eigen::VectorXd queryPoint );

/////////////////////// Within Range ///////////////////////

// Adds the node to the list if it is not already there
bool addToRangeList( JList* S, KDTreeNode* node, double key );

// Pops the range list
void popFromRangeList( JList* S, KDTreeNode* t, double* k );

// Empty the range list
void emptyRangeList( JList* S );

// Finds all nodes within range of the queryPoint in the subtree starting
// at root and also their distance squared. This data is stored in nodeList
// The nodeList may also contain nodes before this function is called
bool kdFindWithinRangeInSubtree( std::string distanceFunction, KDTreeNode* root,
                                 double range, Eigen::VectorXd queryPoint,
                                 JList* nodeList );

// Returns all nodes within range of queryPoint and also their distances
// This data is contained in the JList at S
void kdFindWithinRange( JList* S, KDTree* tree, double range, Eigen::VectorXd queryPoint );

// Inserts a new point into the tree (used only for debugging)
void kdInsert( KDTree* tree, Eigen::VectorXd a )
{
    KDTreeNode* N = new KDTreeNode();
    N->position = a;
    kdInsert(tree, N);
}

#endif // KDTREE_H
