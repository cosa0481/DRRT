/* kdtree.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef KDTREE_H
#define KDTREE_H


#include <DRRT/ghostPoint.h>
#include <DRRT/drrt_data_structures.h>
#include <DRRT/drrt_distance_functions.h>


// A KD-Tree data structure that stores nodes of type T
class KDTree {
public:
    int d;                      // the number of dimensions in the space (5)

    // distance function to use
    double (*distanceFunction)(Eigen::VectorXd a, Eigen::VectorXd b);

    int treeSize;              // the number of nodes in the KD-Tree

    int numWraps;              // the total number of dimensions that wrap
    Eigen::VectorXi wraps; // a vector of length d containing a list of
                           // all the dimensions that wrapAround
    Eigen::VectorXd wrapPoints; // space is assumed to start at 0 and end at
                                // wrapPoints[i] along dimension wraps[i]
    std::shared_ptr<KDTreeNode> root;   // the root node

    // Constructors
    KDTree(int _d, Eigen::VectorXi _wraps, Eigen::VectorXd _wrapPoints)
        :   d(_d), distanceFunction(0), treeSize(0),
            numWraps(_wraps.size()), wraps(_wraps), wrapPoints(_wrapPoints)
    {}
    KDTree(int _d)
        :   d(_d), distanceFunction(0), treeSize(0), numWraps(0)
    {}
    KDTree()
        :  d(0), distanceFunction(0), treeSize(0), numWraps(0)
    {}

    // Setter for distanceFunction
    void setDistanceFunction(double(*func)(Eigen::VectorXd a,
                                           Eigen::VectorXd b))
    { distanceFunction = func; }

    // Prints the tree from the node starting with indent=0
    void printTree(std::shared_ptr<KDTreeNode> node,
                   int indent=0, char type=' ');

    // Inserts a new node into the tree
    bool kdInsert(std::shared_ptr<KDTreeNode> node);

    /////////////////////// Nearest ///////////////////////

    // Returns the nearest node to the queryPoint in the subtree starting
    // at root and also its distance. It takes a suggestion for a possible
    // closest node and uses that if it is best
    bool kdFindNearestInSubtree(std::shared_ptr<KDTreeNode> &nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                std::shared_ptr<KDTreeNode> &root,
                                Eigen::VectorXd queryPoint,
                         std::shared_ptr<KDTreeNode> &suggestedClosestNode,
                                double suggestedClosestDist);

    // Returns the nearest node to the queryPoint and also its distance
    bool kdFindNearest(std::shared_ptr<KDTreeNode> &nearestNode,
                       std::shared_ptr<double> nearestNodeDist,
                       Eigen::VectorXd queryPoint);

    // Returns the nearest node to queryPoint in the subtree starting at
    // the root and also its distance. It also takes a suggestion for a
    // possible closest node (and uses that if it is best)
    bool kdFindNearestinSubtreeWithGuess(
                                std::shared_ptr<KDTreeNode> nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                         std::shared_ptr<KDTreeNode> root,
                                         Eigen::VectorXd queryPoint,
                      std::shared_ptr<KDTreeNode> suggestedClosestNode,
                                         double suggestedClosestDist);

    // Returns the nearest node to the queryPoint and also its distance
    // Instead of starting at the root, starts at guess
    bool kdFindNearestWithGuess(std::shared_ptr<KDTreeNode> nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                Eigen::VectorXd queryPoint,
                                std::shared_ptr<KDTreeNode> guess);

    /////////////////////// K Nearest ///////////////////////

    // Adds the node to the heap if there is space in the current heap
    // without growing past k, otherwise the current top is removed
    // and then the top is returned

    std::shared_ptr<KDTreeNode> addToKNNHeap(std::shared_ptr<BinaryHeap> H,
                                             std::shared_ptr<KDTreeNode> node,
                                             double key, int k);

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
    bool kdFindKNearestInSubtree(std::shared_ptr<KDTreeNode> farthestNode,
                                 std::shared_ptr<double> farthestNodeDist,
                                 std::shared_ptr<KDTreeNode> root,
                                 int k,
                                 Eigen::VectorXd queryPoint,
                                 std::shared_ptr<BinaryHeap> nearestHeap);

    // Returns the K nearest nodes to queryPoint and olso their distances
    // Note that they are not sorted but are in reverse heap order
    std::vector<std::shared_ptr<KDTreeNode>> kdFindKNearest(int k,
                                                Eigen::VectorXd queryPoint);

    /////////////////////// Within Range ///////////////////////

    // Adds the node to the list if it is not already there
    bool addToRangeList(std::shared_ptr<JList> S,
                        std::shared_ptr<KDTreeNode> node,
                        double key);

    // Pops the range list
    void popFromRangeList(std::shared_ptr<JList> S,
                          std::shared_ptr<KDTreeNode> t,
                          std::shared_ptr<double> k);

    // Empty the range list
    void emptyRangeList(std::shared_ptr<JList> &S);

    // Finds all nodes within range of the queryPoint in the subtree starting
    // at root and also their distance squared. This data is stored in nodeList
    // The nodeList may also contain nodes before this function is called
    bool kdFindWithinRangeInSubtree(std::shared_ptr<KDTreeNode> root,
                                    double range,
                                    Eigen::VectorXd queryPoint,
                                    std::shared_ptr<JList> nodeList);

    // Returns all nodes within range of queryPoint and also their distances
    // This data is contained in the JList at S
    /* This function walks down the kdtree as if it were inserting
     * queryPoint to find where it would be inserted. It then walks back
     * up the tree by looking at parents and checking the "hyperplanedist"
     * It also checks further down the kdtree from where the point would
     * be inserted
     */
    void kdFindWithinRange(std::shared_ptr<JList> S, double range,
                           Eigen::VectorXd queryPoint);

    // Returns all nodes within range of queryPoint and also their distance.
    // They are returned in a list with elements of type KDTreeNode
    // THe list to be used is passed in so additional points can be added
    // to it (e.ge, if we want to have one list containing the points that
    // are close to a couple of different points X1,..,Xn, then call this
    // for X2,...,Xn after first calling kdFindWithinRange for X1
    void kdFindMoreWithinRange(std::shared_ptr<JList> S, double range,
                               Eigen::VectorXd queryPoint);

    // Inserts a new point into the tree (used only for debugging)
    void kdInsert(Eigen::VectorXd a);
};

#endif // KDTREE_H
