#ifndef KD_HEAPNODE_H
#define KD_HEAPNODE_H

#include <DRRT/heapnode.h>
#include <DRRT/kdnode.h>

class KdHeapNode : public HeapNode
{
public:
    std::shared_ptr<Kdnode> data_;

    KdHeapNode(std::shared_ptr<Kdnode> &n) : HeapNode(n->GetLmc()), data_(n)
    { SetEmpty(false); }
    KdHeapNode() : HeapNode() {}
};

#endif // KD_HEAPNODE_H
