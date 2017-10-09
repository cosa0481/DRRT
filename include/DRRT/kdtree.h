#ifndef KDTREE_H
#define KDTREE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode_listnode.h>
#include <DRRT/range_listnode.h>
#include <DRRT/ghost_point.h>

class KdTree : public std::enable_shared_from_this<KdTree>
{
public:
    std::mutex tree_mutex_;

    int dimensions_;
    int size_;
    Kdnode_ptr root_;
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

    std::shared_ptr<KdTree> GetSharedPointer() { return shared_from_this(); }

    void Print(Kdnode_ptr node, int indent=0, char type=' ');
    void GetNodeAt(Kdnode_ptr &node, Eigen::VectorXd pos);
    bool Insert(Kdnode_ptr &node);

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

    void AddToRangeList(RangeList_ptr list, Kdnode_ptr &node, double value);
    double PopFromRangeList(RangeList_ptr list, Kdnode_ptr &node);
    void EmptyRangeList(RangeList_ptr list);

    // Following 3 functions return a pointer to a KdnodeList containing the Kdnodes within range
    RangeList_ptr FindWithinRange(double range, Eigen::VectorXd query);
    RangeList_ptr FindWithinRangeInSubtree(Kdnode_ptr sub_root, double range,
                                           Eigen::VectorXd query, RangeList_ptr range_list);
    RangeList_ptr FindMoreWithinRange(double range, Eigen::VectorXd query, RangeList_ptr range_list);
};

typedef std::shared_ptr<KdTree> KdTree_ptr;

#endif // KDTREE_H
