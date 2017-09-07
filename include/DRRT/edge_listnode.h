#ifndef EDGE_LISTNODE_H
#define EDGE_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/edge.h>

class EdgeListNode : public ListNode
{
    bool is_empty_;
    bool in_list_;

public:
    std::shared_ptr<EdgeListNode> child_;
    std::shared_ptr<EdgeListNode> parent_;
    std::shared_ptr<Edge> data_;

    EdgeListNode(std::shared_ptr<Edge> &e) : data_(e) { SetEmpty(false); }
    EdgeListNode() : is_empty_(true) {}

    void GetData(std::shared_ptr<Edge> &edge) { edge = data_; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

#endif // EDGE_LISTNODE_H
