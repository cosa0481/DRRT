/* jlist.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef JLIST_H
#define JLIST_H

#include <DRRT/obstacle.h>

class KDTreeNode;
class Edge;

// A JList node_ (not that key_ is unused for JList operations,
// but it is often helpful to have a key_ value associated
// with data)
class JListNode{
public:
    std::shared_ptr<JListNode> child_;
    std::shared_ptr<JListNode> parent_;
    std::shared_ptr<KDTreeNode> node_;
    std::shared_ptr<Edge> edge_;
    double key_ = 0.0;

    // Corstructor
    JListNode() : key_(-1.0) {}
    JListNode(std::shared_ptr<KDTreeNode> &t) : node_(t) {}
    JListNode(std::shared_ptr<Edge> &e) : edge_(e) {}
};

// A simple JList
class JList{
public:
    std::shared_ptr<JListNode> front_;
    std::shared_ptr<JListNode> back_;
    std::shared_ptr<JListNode> bound_;   // bounds either side of the list
    int length_;
    bool use_nodes_;    // flag for indicating whether this JList
                        // is one of Edges or KDTreeNodes. True
                        // if JList uses KDTreeNodes

    // Constructor
    JList( bool node_flag ) : use_nodes_(node_flag)
    {
        std::shared_ptr<JListNode> end_node = std::make_shared<JListNode>();
        end_node->child_ = end_node;
        end_node->parent_ = end_node;

        front_ = end_node;
        back_ = end_node;
        bound_ = end_node;
        length_ = 0;
    }

    // Functions
    bool JListContains(std::shared_ptr<KDTreeNode> &t,
                       std::shared_ptr<JListNode> &i);
    void JListPush( std::shared_ptr<KDTreeNode> &t );
    void JListPush( std::shared_ptr<Edge> &e );
    void JListPush( std::shared_ptr<KDTreeNode> &t, double k );
    void JListPush( std::shared_ptr<Edge> &e, double k );
    void JListTop( std::shared_ptr<Edge> &e );
    void JListTop( std::shared_ptr<KDTreeNode> &t );
    void JListTopKey(std::shared_ptr<KDTreeNode> &n,
                     std::shared_ptr<double> k );
    void JListTopKey( std::shared_ptr<Edge> &e, std::shared_ptr<double> k );
    void JListPop( std::shared_ptr<KDTreeNode> &t );
    void JListPop( std::shared_ptr<Edge> &e );
    void JListPopKey(std::shared_ptr<KDTreeNode> &n,
                     std::shared_ptr<double> k);
    void JListPopKey(std::shared_ptr<Edge> &e, std::shared_ptr<double> k);
    bool JListRemove(std::shared_ptr<JListNode> &node);
    void JListPrint();
    Eigen::Matrix<double, Eigen::Dynamic, 2> JListAsMatrix();
    void JListEmpty();
};

#endif // JLIST_H
