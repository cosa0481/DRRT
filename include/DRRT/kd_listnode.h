#ifndef KD_LISTNODE_H
#define KD_LISTNODE_H

#include <DRRT/listnode.h>
#include <DRRT/kdnode.h>

class KdListNode : public ListNode
{
public:
    std::shared_ptr<Kdnode> data_;

    KdListNode(std::shared_ptr<Kdnode> &n) : data_(n) { SetEmpty(false); }
    KdListNode() : ListNode() {}

    void GetData(std::shared_ptr<Kdnode> node) { node = data_; }
};

#endif // KD_LISTNODE_H
