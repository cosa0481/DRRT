#ifndef LISTNODE_NEW_H
#define LISTNODE_NEW_H

#include <DRRT/libraries.h>

class ListNode
{
    bool is_empty_;

public:
    std::shared_ptr<ListNode> child_;

    ListNode() : is_empty_(true) {}

    virtual void GetData() {}
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
};

#endif // LISTNODE_NEW_H
