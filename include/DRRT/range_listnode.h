#ifndef RANGE_LISTNODE_H
#define RANGE_LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode_listnode.h>

class RangeListNode
{
    bool is_empty_;
    bool in_list_;

    Kdnode_ptr data_;
    double value_;

    std::shared_ptr<RangeListNode> child_;
    std::shared_ptr<RangeListNode> parent_;

public:
    RangeListNode(Kdnode_ptr &node, double dist) : is_empty_(false), in_list_(false), data_(node), value_(dist) {}
    RangeListNode() : is_empty_(true), in_list_(false) {}

    double GetData(Kdnode_ptr &node) { node = data_; return value_; }
    std::shared_ptr<RangeListNode> GetChild() { return child_; }
    std::shared_ptr<RangeListNode> GetParent() { return parent_; }
    void SetChild(std::shared_ptr<RangeListNode> new_child) { child_ = new_child; }
    void SetParent(std::shared_ptr<RangeListNode> new_parent) { parent_ = new_parent; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

typedef std::shared_ptr<RangeListNode> RangeListNode_ptr;
typedef List<RangeListNode> RangeList;
typedef std::shared_ptr<RangeList> RangeList_ptr;

#endif // RANGE_LISTNODE_H
