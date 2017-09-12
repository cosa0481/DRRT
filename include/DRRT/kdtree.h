#ifndef KDTREE_H
#define KDTREE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode_listnode.h>

class KdTree {
public:
    std::mutex tree_mutex_;

    int dimensions_;
    int size_;
    Kdnode_ptr root;
    std::vector<Kdnode_ptr> nodes_;

    int num_wraps_;
    Eigen::VectorXi wraps_;
    Eigen::VectorXd wrap_points_;


    KdTree(int dims, Eigen::VectorXi wraps, Eigen::VectorXd wrappoints)
        : dimensions_(dims), size_(0), nodes_(std::vector<Kdnode_ptr>()),
          num_wraps_(wraps.size()), wraps_(wraps), wrap_points_(wrappoints)
    {}

    KdTree() : dimensions_(0), size_(0), nodes_(std::vector<Kdnode_ptr>()),
        num_wraps_(0), wraps_(Eigen::VectorXi(NUM_DIM)), wrap_points_(Eigen::VectorXd(NUM_DIM))
    {}

    void Print(Kdnode_ptr start_node, int indent=0, char type=' ');
    void GetNodeAt(Kdnode_ptr &node, Eigen::VectorXd pos);
    bool KdInsert(Kdnode_ptr &node);

    // KdTree functions

    // Following 4 functions return the distance to the 'nearest' node
    // The nearest node is populated with the nearest node in the KdTree
    double FindNearest(Kdnode_ptr &nearest, Eigen::VectorXd query);
    double FindNearestInSubtree(Kdnode_ptr &nearest, Kdnode_ptr sub_root,
                                Eigen::VectorXd query,
                                Kdnode_ptr &suggested, double suggested_dist);
    double FindNearestWithGuess(Kdnode_ptr &nearest, Eigen::VectorXd query, Kdnode_ptr guess);
    double FindNearestInSubtreeWithGuess(Kdnode_ptr &nearest, Kdnode_ptr sub_root,
                                         Eigen::VectorXd query, Kdnode_ptr &suggested,
                                         double suggested_dist);

    // Following 3 functions return a pointer to a KdnodeList containing the Kdnodes within range
    KdnodeList_ptr FindWithinRange(double range, Eigen::VectorXd query);
    KdnodeList_ptr FindWithinRangeInSubtree(Kdnode_ptr sub_root, double range, Eigen::VectorXd query);
    KdnodeList_ptr FindMoreWithinRange(double, Eigen::VectorXd query);
};

typedef std::shared_ptr<KdTree> KdTree_ptr;

#endif // KDTREE_H
