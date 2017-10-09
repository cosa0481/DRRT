#ifndef KD_HEAPNODE_H
#define KD_HEAPNODE_H

#include <DRRT/heapnode.h>
#include <DRRT/kdnode.h>

class KdHeapNode : public HeapNode
{
public:
    Kdnode_ptr data_;

    KdHeapNode(Kdnode_ptr &n) : HeapNode(n->GetCost()), data_(n)
    { SetEmpty(false); }
    KdHeapNode() : HeapNode() {}

    void GetData(Kdnode_ptr &node) { node = data_; }
};

typedef std::shared_ptr<KdHeapNode> KdHeapNode_ptr;

#endif // KD_HEAPNODE_H
