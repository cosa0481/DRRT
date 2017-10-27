#ifndef KDNODE_H
#define KDNODE_H

#include <DRRT/libraries.h>
#include <DRRT/edge_listnode.h>

class Kdnode
{
    bool in_range_list_;
    bool in_priority_queue_;
    bool in_OS_queue_;
    bool in_tree_;
    bool is_movegoal_;

    bool parent_exist_;
    bool lchild_exist_;
    bool rchild_exist_;

    double cost_;
    double lmc_;

    Eigen::VectorXd position_;

    bool rrt_parent_exist_;
    std::shared_ptr<Edge> rrt_parent_edge_;

    EdgeList_ptr out_neighbors_;
    EdgeList_ptr in_neighbors_;

    EdgeList_ptr successor_list_;
    EdgeListNode_ptr successor_list_item_in_parent_;

    EdgeList_ptr initial_out_neighbors_;
    EdgeList_ptr initial_in_neighbors_;

public:
    int split_;
    std::shared_ptr<Edge> temp_edge_;
    std::shared_ptr<Kdnode> parent_;
    std::shared_ptr<Kdnode> lchild_;
    std::shared_ptr<Kdnode> rchild_;

    // Cost constructor
    Kdnode(double cost)
        : in_range_list_(false), in_priority_queue_(false), in_OS_queue_(false),
          in_tree_(false), is_movegoal_(false), parent_exist_(false),
          lchild_exist_(false), rchild_exist_(false), cost_(cost), lmc_(INF),
          rrt_parent_exist_(false),
          rrt_parent_edge_(Edge::NewEdge()),
          out_neighbors_(std::make_shared<EdgeList>()),
          in_neighbors_(std::make_shared<EdgeList>()),
          successor_list_(std::make_shared<EdgeList>()),
          successor_list_item_in_parent_(std::make_shared<EdgeListNode>()),
          initial_out_neighbors_(std::make_shared<EdgeList>()),
          initial_in_neighbors_(std::make_shared<EdgeList>())
    { position_.setZero(NUM_DIM); }

    // Position constructor
    Kdnode(Eigen::VectorXd pos)
        : in_range_list_(false), in_priority_queue_(false), in_OS_queue_(false),
          in_tree_(false), is_movegoal_(false), parent_exist_(false),
          lchild_exist_(false), rchild_exist_(false), cost_(INF), lmc_(INF),
          position_(pos),
          rrt_parent_exist_(false),
          rrt_parent_edge_(Edge::NewEdge()),
          out_neighbors_(std::make_shared<EdgeList>()),
          in_neighbors_(std::make_shared<EdgeList>()),
          successor_list_(std::make_shared<EdgeList>()),
          successor_list_item_in_parent_(std::make_shared<EdgeListNode>()),
          initial_out_neighbors_(std::make_shared<EdgeList>()),
          initial_in_neighbors_(std::make_shared<EdgeList>())
    {}

    // Cost and position constructor
    Kdnode(double cost, Eigen::VectorXd pos)
        : in_range_list_(false), in_priority_queue_(false), in_OS_queue_(false),
          in_tree_(false), is_movegoal_(false), parent_exist_(false),
          lchild_exist_(false), rchild_exist_(false), cost_(cost), lmc_(INF),
          position_(pos),
          rrt_parent_exist_(false),
          rrt_parent_edge_(Edge::NewEdge()),
          out_neighbors_(std::make_shared<EdgeList>()),
          in_neighbors_(std::make_shared<EdgeList>()),
          successor_list_(std::make_shared<EdgeList>()),
          successor_list_item_in_parent_(std::make_shared<EdgeListNode>()),
          initial_out_neighbors_(std::make_shared<EdgeList>()),
          initial_in_neighbors_(std::make_shared<EdgeList>())
    {}

    // Default constructor
    Kdnode()
        : in_range_list_(false), in_priority_queue_(false), in_OS_queue_(false),
          in_tree_(false), is_movegoal_(false), parent_exist_(false),
          lchild_exist_(false), rchild_exist_(false), cost_(INF), lmc_(INF),
          rrt_parent_exist_(false),
          rrt_parent_edge_(Edge::NewEdge()),
          out_neighbors_(std::make_shared<EdgeList>()),
          in_neighbors_(std::make_shared<EdgeList>()),
          successor_list_(std::make_shared<EdgeList>()),
          successor_list_item_in_parent_(std::make_shared<EdgeListNode>()),
          initial_out_neighbors_(std::make_shared<EdgeList>()),
          initial_in_neighbors_(std::make_shared<EdgeList>())
    { position_.setZero(NUM_DIM); }

    bool InRangeList() { return in_range_list_; }
    bool InPriorityQueue() { return in_priority_queue_; }
    bool InOSQueue() { return in_OS_queue_; }
    bool InTree() { return in_tree_; }
    bool IsGoal() { return is_movegoal_; }
    bool ParentExist() { return parent_exist_; }
    bool LChildExist() { return lchild_exist_; }
    bool RChildExist() { return rchild_exist_; }
    double GetCost() { return cost_; }
    double GetLmc() { return lmc_; }
    Eigen::VectorXd GetPosition() { return position_; }
    bool RrtParentExist() { return rrt_parent_exist_; }
    void GetRrtParentEdge(std::shared_ptr<Edge> &edge) { edge = rrt_parent_edge_; }
    EdgeList_ptr GetOutNeighbors() { return out_neighbors_; }
    EdgeList_ptr GetInNeighbors() { return in_neighbors_; }
    EdgeList_ptr GetSuccessorList() { return successor_list_; }
    EdgeListNode_ptr GetSuccessorInParent() { return successor_list_item_in_parent_; }
    EdgeList_ptr GetInitialOutNeighbors() { return initial_out_neighbors_; }
    EdgeList_ptr GetInitialInNeighbors() { return initial_in_neighbors_; }

    void SetInRangeList(bool inlist) { in_range_list_ = inlist; }
    void SetInPriorityQueue(bool inqueue) { in_priority_queue_ = inqueue; }
    void SetInOSQueue(bool inqueue) { in_OS_queue_ = inqueue; }
    void SetInTree(bool intree) { in_tree_ = intree; }
    void SetIsGoal(bool isgoal) { is_movegoal_ = isgoal; }
    void SetParentExist(bool exist) { parent_exist_ = exist; }
    void SetLChildExist(bool exist) { lchild_exist_ = exist; }
    void SetRChildExist(bool exist) { rchild_exist_ = exist; }
    void SetCost(double new_cost) { cost_ = new_cost; }
    void SetLmc(double new_lmc) { lmc_ = new_lmc; }
    void SetPosition(Eigen::VectorXd pos) { position_ = pos; }
    void SetRrtParentExist(bool exist) { rrt_parent_exist_ = exist; }
    void SetRrtParentEdge(std::shared_ptr<Edge> parentedge) { rrt_parent_edge_ = parentedge; }
    void SetOutNeighbors(EdgeList_ptr outs) { out_neighbors_ = outs; }
    void SetInNeighbors(EdgeList_ptr ins) { in_neighbors_ = ins; }
    void SetSuccessorList(EdgeList_ptr slist) { successor_list_ = slist; }
    void SetSuccessorInParent(EdgeListNode_ptr sip) { successor_list_item_in_parent_ = sip; }
    void SetInitialOutNeighbors(EdgeList_ptr outs) { initial_out_neighbors_ = outs; }
    void SetInitialInNeighbors(EdgeList_ptr ins) { initial_in_neighbors_ = ins; }

    inline Eigen::VectorXd Saturate(Eigen::VectorXd position,
                             Eigen::VectorXd closest_point,
                             double delta, double dist)
    { return SaturateDubins(position, closest_point, delta, dist); }
};

typedef std::shared_ptr<Kdnode> Kdnode_ptr;

#endif // KDNODE_H
