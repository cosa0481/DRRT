#ifndef KD_LISTNODE_H
#define KD_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode.h>
#include <DRRT/list.h>

class KdListNode
{
    bool is_empty_;
    bool in_list_;

public:
    std::shared_ptr<KdListNode> child_;
    std::shared_ptr<KdListNode> parent_;
    Kdnode_ptr data_;

    KdListNode(Kdnode_ptr &n) : data_(n) { SetEmpty(false); }
    KdListNode() : is_empty_(true) {}

    void GetData(Kdnode_ptr &node) { node = data_; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

typedef std::shared_ptr<KdListNode> KdListNode_ptr;
typedef List<KdListNode> KdnodeList;
typedef std::shared_ptr<KdnodeList> KdnodeList_ptr;

#endif // KD_LISTNODE_H
