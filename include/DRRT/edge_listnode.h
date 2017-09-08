#ifndef EDGE_LISTNODE_H
#define EDGE_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/edge.h>
#include <DRRT/list.h>

class EdgeListNode
{
    bool is_empty_;
    bool in_list_;

public:
    std::shared_ptr<EdgeListNode> child_;
    std::shared_ptr<EdgeListNode> parent_;
    Edge_ptr data_;

    EdgeListNode(Edge_ptr &e) : data_(e) { SetEmpty(false); }
    EdgeListNode() : is_empty_(true) {}

    void GetData(Edge_ptr &edge) { edge = data_; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

typedef std::shared_ptr<EdgeListNode> EdgeListNode_ptr;
typedef List<EdgeListNode> EdgeList;
typedef std::shared_ptr<EdgeList> EdgeList_ptr;


#endif // EDGE_LISTNODE_H
