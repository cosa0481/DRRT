#ifndef KDTREENODE_H
#define KDTREENODE_H

#include <DRRT/jlist.h>

#define PI 3.1415926536

class Edge;

/* Node that can be used in the KDTree, where T is the type of
 * data used to measure distance along each dimension. Other nodes
 * can also be used as long as they have these fields that are
 * initialized as follows by a default constructor and the parent
 * and children types are the same as the node's type itself.
 */
class KDTreeNode{
public:
    // Data used for KD Tree
    bool kd_in_tree_;          // set to true if this node is in the kd-tree
    bool kd_parent_exist_;     // set to true if parent in the tree is used
    bool kd_child_L_exist_;     // set to true if left child in the tree is used
    bool kd_child_R_exist_;     // set to true if right child in the tree is used

    // Data used for heap in KNN-search
    int heap_index_;    // named such to allow the use of default heap functions
    bool in_heap_;      // ditto
    double dist_;      // ditto, this will hold the distance

    // More data used for KD Tree
    Eigen::VectorXd position_; // a d x 1 array of the dimensions of the space
    int kd_split_;              // the dimension used for splitting at this node
    std::shared_ptr<KDTreeNode> kd_parent_;   // parent in the tree
    std::shared_ptr<KDTreeNode> kd_child_L_;   // left child in the tree
    std::shared_ptr<KDTreeNode> kd_child_R_;   // right child in the tree

    // RRT
    bool rrt_parent_used_;     // flag for if this node has a parent
    std::shared_ptr<Edge> rrt_parent_edge_;    // edge to the node that is
                                            // this node's parent

    // RRT*
    double rrt_tree_cost_;     // the cost to get to the root through the tree

    // RRT#
    std::shared_ptr<JList> rrt_neighbors_out_;  // edges in the graph that
                                             // can be reached
                                             // from this node
    std::shared_ptr<JList> rrt_neighbors_in_;   // edges in the graph that
                                             // reach this node

    int priority_queue_index_;     // index in the queue
    bool in_priority_queue_;       // flag for in the queue

    double rrt_LMC_;              // locally minimum cost (1-step look ahead)
    double rrt_H_;        // the heuristic estimate of the distance to the goal
    std::shared_ptr<Edge> temp_edge_;  // this is a temporary storage
                                     // location to avoid
                            // calculating the same trajectory multiple times

    // RRTx (his idea)
    std::shared_ptr<JList> successor_list_; // edges to nodes that use
                                          // this node as their parent
    std::shared_ptr<JList> initial_neighbor_list_out_; // edges to nodes in
                                                   // the original ball
                                   // that can be reached bfrom this node
    std::shared_ptr<JList> initial_neighbor_list_in_;  // edges to nodes in
                                                   // the original ball
                                                // that can reach this node

    bool in_OS_queue_;     // flag for in the OS queue
    bool is_move_goal_;    // true if this is move goal (robot pose)

    // pointer to the list node in the parent's successor list that
    // holds parent's edge to this node
    std::shared_ptr<JListNode> successor_list_item_in_parent_;


    // Constructors
    KDTreeNode() : kd_in_tree_(false), kd_parent_exist_(false), kd_child_L_exist_(false),
        kd_child_R_exist_(false), heap_index_(-1), in_heap_(false), dist_(-1),
        rrt_parent_used_(false), rrt_neighbors_out_(std::make_shared<JList>(false)),
        rrt_neighbors_in_(std::make_shared<JList>(false)), priority_queue_index_(-1),
        in_priority_queue_(false), successor_list_(std::make_shared<JList>(false)),
        initial_neighbor_list_out_(std::make_shared<JList>(false)),
        initial_neighbor_list_in_(std::make_shared<JList>(false)),
        in_OS_queue_(false), is_move_goal_(false)
    {
        position_.setZero();
    }
    KDTreeNode(float d) :  kd_in_tree_(false), kd_parent_exist_(false),
        kd_child_L_exist_(false), kd_child_R_exist_(false), heap_index_(-1),
        in_heap_(false), dist_(d), rrt_parent_used_(false),
        rrt_neighbors_out_(std::make_shared<JList>(false)),
        rrt_neighbors_in_(std::make_shared<JList>(false)),
        priority_queue_index_(-1), in_priority_queue_(false),
        successor_list_(std::make_shared<JList>(false)),
        initial_neighbor_list_out_(std::make_shared<JList>(false)),
        initial_neighbor_list_in_(std::make_shared<JList>(false)),
        in_OS_queue_(false), is_move_goal_(false)
    {
        position_.setZero();
    }
    KDTreeNode(float d, Eigen::VectorXd pos) :  kd_in_tree_(false),
        kd_parent_exist_(false), kd_child_L_exist_(false), kd_child_R_exist_(false),
        heap_index_(-1), in_heap_(false), dist_(d), position_(pos),
        rrt_parent_used_(false), rrt_neighbors_out_(std::make_shared<JList>(false)),
        rrt_neighbors_in_(std::make_shared<JList>(false)), priority_queue_index_(-1),
        in_priority_queue_(false), successor_list_(std::make_shared<JList>(false)),
        initial_neighbor_list_out_(std::make_shared<JList>(false)),
        initial_neighbor_list_in_(std::make_shared<JList>(false)),
        in_OS_queue_(false), is_move_goal_(false)
    {}
    KDTreeNode(Eigen::VectorXd pos) : kd_in_tree_(false), kd_parent_exist_(false),
        kd_child_L_exist_(false), kd_child_R_exist_(false), heap_index_(-1),
        in_heap_(false), dist_(INFINITY), position_(pos), rrt_parent_used_(false),
        rrt_neighbors_out_(std::make_shared<JList>(false)),
        rrt_neighbors_in_(std::make_shared<JList>(false)),
        priority_queue_index_(-1), in_priority_queue_(false),
        successor_list_(std::make_shared<JList>(false)),
        initial_neighbor_list_out_(std::make_shared<JList>(false)),
        initial_neighbor_list_in_(std::make_shared<JList>(false)),
        in_OS_queue_(false), is_move_goal_(false)
    {}

    // Copies values in KDTreeNode other to this object
    KDTreeNode(KDTreeNode &other) :
        kd_in_tree_(other.kd_in_tree_),
        kd_parent_exist_(other.kd_parent_exist_),
        kd_child_L_exist_(other.kd_child_L_exist_),
        kd_child_R_exist_(other.kd_child_R_exist_),
        heap_index_(other.heap_index_),
        in_heap_(other.in_heap_),
        dist_(other.dist_),
        position_(other.position_),
        kd_split_(other.kd_split_),
        kd_parent_(other.kd_parent_),
        kd_child_L_(other.kd_child_L_),
        kd_child_R_(other.kd_child_R_),
        rrt_parent_used_(other.rrt_parent_used_),
        rrt_parent_edge_(other.rrt_parent_edge_),
        rrt_tree_cost_(other.rrt_tree_cost_),
        rrt_neighbors_out_(other.rrt_neighbors_out_),
        rrt_neighbors_in_(other.rrt_neighbors_in_),
        priority_queue_index_(other.priority_queue_index_),
        in_priority_queue_(other.in_priority_queue_),
        rrt_LMC_(other.rrt_LMC_),
        rrt_H_(other.rrt_H_),
        temp_edge_(other.temp_edge_),
        successor_list_(other.successor_list_),
        initial_neighbor_list_out_(other.initial_neighbor_list_out_),
        initial_neighbor_list_in_(other.initial_neighbor_list_in_),
        in_OS_queue_(other.in_OS_queue_),
        is_move_goal_(other.is_move_goal_),
        successor_list_item_in_parent_(other.successor_list_item_in_parent_)
    {}
};

#endif // KDTREENODE_H
