#ifndef EDGE_LISTNODE_H
#define EDGE_LISTNODE_H

#include <DRRT/listnode.h>
#include <DRRT/edge.h>

class EdgeListNode : public ListNode
{
public:
    std::shared_ptr<Edge> data_;

    EdgeListNode(std::shared_ptr<Edge> &e) : data_(e) { SetEmpty(false); }
    EdgeListNode() : ListNode() {}

    void GetData(std::shared_ptr<Edge> edge) { edge = data_; }
};

#endif // EDGE_LISTNODE_H
