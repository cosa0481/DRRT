#ifndef KD_LISTNODE_H
#define KD_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode.h>

class KdListNode
{
    bool is_empty_;
    bool in_list_;

public:
    std::shared_ptr<KdListNode> child_;
    std::shared_ptr<KdListNode> parent_;
    std::shared_ptr<Kdnode> data_;

    KdListNode(std::shared_ptr<Kdnode> &n) : data_(n) { SetEmpty(false); }
    KdListNode() : is_empty_(true) {}

    void GetData(std::shared_ptr<Kdnode> &node) { node = data_; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

#endif // KD_LISTNODE_H
