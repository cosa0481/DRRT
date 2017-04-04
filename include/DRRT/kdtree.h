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
    std::mutex tree_mutex_;
    std::vector<std::shared_ptr<KDTreeNode>> nodes_;

    int dimensions_;                       // the number of dimensions in the space (5)

    // distance function to use
    double (*distanceFunction)(Eigen::VectorXd a, Eigen::VectorXd b);

    int tree_size_;               // the number of nodes in the KD-Tree

    int num_wraps_;               // the total number of dimensions that wrap
    Eigen::VectorXi wraps_;      // a vector of length dimensions_ containing a list of
                                // all the dimensions that wrapAround
    Eigen::VectorXd wrap_points_; // space is assumed to start at 0 and end at
                                // wrap_points_[i] along dimension wraps_[i]
    std::shared_ptr<KDTreeNode> root;   // the root node

    // Constructors
    KDTree(int _d, Eigen::VectorXi _wraps, Eigen::VectorXd _wrapPoints)
        :   dimensions_(_d), distanceFunction(0), tree_size_(0),
            num_wraps_(_wraps.size()), wraps_(_wraps), wrap_points_(_wrapPoints)
    { nodes_ = std::vector<std::shared_ptr<KDTreeNode>>(); }

    KDTree(int _d)
        :   dimensions_(_d), distanceFunction(0), tree_size_(0), num_wraps_(0)
    { nodes_ = std::vector<std::shared_ptr<KDTreeNode>>(); }

    KDTree()
        :  dimensions_(0), distanceFunction(0), tree_size_(0), num_wraps_(0)
    { nodes_ = std::vector<std::shared_ptr<KDTreeNode>>(); }

    // Setter for distanceFunction
    void SetDistanceFunction(double(*func)(Eigen::VectorXd a,
                                           Eigen::VectorXd b))
    { distanceFunction = func; }

    // Adds a node to the JList for the visualizer
    void AddVizNode(std::shared_ptr<KDTreeNode> node);

    // Removes a node from the visualizer node list
    void RemoveVizNode(std::shared_ptr<KDTreeNode> &node);

    // Prints the tree from the node starting with indent=0
    void PrintTree(std::shared_ptr<KDTreeNode> node,
                   int indent=0, char type=' ');

    // Gets node in tree at position if present
    void GetNodeAt(Eigen::VectorXd pos,
                   std::shared_ptr<KDTreeNode>& node);

    // Inserts a new node into the tree
    bool KDInsert(std::shared_ptr<KDTreeNode> &node);

    /////////////////////// Nearest ///////////////////////

    // Returns the nearest node to the queryPoint in the subtree starting
    // at root and also its distance. It takes a suggestion for a possible
    // closest node and uses that if it is best
    bool KDFindNearestInSubtree(std::shared_ptr<KDTreeNode> &nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                std::shared_ptr<KDTreeNode> &root,
                                Eigen::VectorXd queryPoint,
                         std::shared_ptr<KDTreeNode> &suggestedClosestNode,
                                double suggestedClosestDist);

    // Returns the nearest node to the queryPoint and also its distance
    bool KDFindNearest(std::shared_ptr<KDTreeNode> &nearestNode,
                       std::shared_ptr<double> nearestNodeDist,
                       Eigen::VectorXd queryPoint);

    // Returns the nearest node to queryPoint in the subtree starting at
    // the root and also its distance. It also takes a suggestion for a
    // possible closest node (and uses that if it is best)
    bool KDFindNearestinSubtreeWithGuess(
                                std::shared_ptr<KDTreeNode> nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                         std::shared_ptr<KDTreeNode> root,
                                         Eigen::VectorXd queryPoint,
                      std::shared_ptr<KDTreeNode> suggestedClosestNode,
                                         double suggestedClosestDist);

    // Returns the nearest node to the queryPoint and also its distance
    // Instead of starting at the root, starts at guess
    bool KDFindNearestWithGuess(std::shared_ptr<KDTreeNode> nearestNode,
                                std::shared_ptr<double> nearestNodeDist,
                                Eigen::VectorXd queryPoint,
                                std::shared_ptr<KDTreeNode> guess);

    /////////////////////// K Nearest ///////////////////////

    // Adds the node to the heap if there is space in the current heap
    // without growing past k, otherwise the current top is removed
    // and then the top is returned

    std::shared_ptr<KDTreeNode> AddToKNNHeap(std::shared_ptr<BinaryHeap> H,
                                             std::shared_ptr<KDTreeNode> node,
                                             double key, int k);

    /* Finds the K nearest nodes to queryPoint in the subtree starting
     * at root. Note that this return data is stored in the nearestHeap,
     * and the heap may also contain nodes before this function is called
     * explicitly. Returns the node of the nearest set that is FARTHEST
     * from the queryPoint along with its distance. Assumes that there
     * is at least one node in the heap to begin with. IF this is the
     * first call to this function (e.g. from KDFindKNearest) then it
     * is also assumed that a dummy node has been added that has an INF
     * key value. This makes things easier with checking that all K slots
     * are used during the recursion
     */
    bool KDFindKNearestInSubtree(std::shared_ptr<KDTreeNode> farthestNode,
                                 std::shared_ptr<double> farthestNodeDist,
                                 std::shared_ptr<KDTreeNode> root,
                                 int k,
                                 Eigen::VectorXd queryPoint,
                                 std::shared_ptr<BinaryHeap> nearestHeap);

    // Returns the K nearest nodes to queryPoint and olso their distances
    // Note that they are not sorted but are in reverse heap order
    std::vector<std::shared_ptr<KDTreeNode>> KDFindKNearest(int k,
                                                Eigen::VectorXd queryPoint);

    /////////////////////// Within Range ///////////////////////

    // Adds the node to the list if it is not already there
    bool AddToRangeList(std::shared_ptr<JList> &S,
                        std::shared_ptr<KDTreeNode> &node,
                        double key);

    // Pops the range list
    void PopFromRangeList(std::shared_ptr<JList> &S,
                          std::shared_ptr<KDTreeNode> &t,
                          std::shared_ptr<double> k);

    // Empty the range list
    void EmptyRangeList(std::shared_ptr<JList> &S);

    // Finds all nodes within range of the queryPoint in the subtree starting
    // at root and also their distance squared. This data is stored in nodeList
    // The nodeList may also contain nodes before this function is called
    bool KDFindWithinRangeInSubtree(std::shared_ptr<KDTreeNode> &root,
                                    double range,
                                    Eigen::VectorXd queryPoint,
                                    std::shared_ptr<JList> &nodeList);

    // Returns all nodes within range of queryPoint and also their distances
    // This data is contained in the JList at S
    /* This function walks down the kdtree as if it were inserting
     * queryPoint to find where it would be inserted. It then walks back
     * up the tree by looking at parents and checking the "hyperplanedist"
     * It also checks further down the kdtree from where the point would
     * be inserted
     */
    void KDFindWithinRange(std::shared_ptr<JList> &S, double range,
                           Eigen::VectorXd queryPoint);

    // Returns all nodes within range of queryPoint and also their distance.
    // They are returned in a list with elements of type KDTreeNode
    // THe list to be used is passed in so additional points can be added
    // to it (e.ge, if we want to have one list containing the points that
    // are close to a couple of different points X1,..,Xn, then call this
    // for X2,...,Xn after first calling KDFindWithinRange for X1
    void KDFindMoreWithinRange(std::shared_ptr<JList> &S, double range,
                               Eigen::VectorXd queryPoint);

    // Inserts a new point into the tree (used only for debugging)
    void KDInsert(Eigen::VectorXd a);
};

#endif // KDTREE_H
