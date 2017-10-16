#ifndef KD_LISTNODE_H
#define KD_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode.h>
#include <DRRT/list.h>

class KdnodeListNode
{
    bool is_empty_;
    bool in_list_;

    std::shared_ptr<KdnodeListNode> child_;
    std::shared_ptr<KdnodeListNode> parent_;
    Kdnode_ptr data_;

public:
    KdnodeListNode(Kdnode_ptr &n) : is_empty_(false), in_list_(false), data_(n) {}
    KdnodeListNode() : is_empty_(true), in_list_(false) {}

    void GetData(Kdnode_ptr &node) { node = data_; }
    std::shared_ptr<KdnodeListNode> GetChild() { return child_; }
    std::shared_ptr<KdnodeListNode> GetParent() { return parent_; }
    void SetChild(std::shared_ptr<KdnodeListNode> new_child) { child_ = new_child; }
    void SetParent(std::shared_ptr<KdnodeListNode> new_parent) { parent_ = new_parent; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

typedef std::shared_ptr<KdnodeListNode> KdnodeListNode_ptr;
typedef List<KdnodeListNode> KdnodeList;
typedef std::shared_ptr<KdnodeList> KdnodeList_ptr;

#endif // KD_LISTNODE_H
