#ifndef NEIGHBOR_ITERATOR_H
#define NEIGHBOR_ITERATOR_H

#include <DRRT/libraries.h>
#include <DRRT/kdnode_listnode.h>
#include <DRRT/edge_listnode.h>

typedef struct RrtNodeNeighborIterator {
    Kdnode_ptr node;
    int list_flag;
    EdgeListNode_ptr current_item;
    RrtNodeNeighborIterator(Kdnode_ptr &this_node) : node(this_node), list_flag(0) {}

    void ResetIterator() { list_flag = 0; }
} RrtNodeNeighborIterator;
typedef std::shared_ptr<RrtNodeNeighborIterator> RrtNodeNeighborIterator_ptr;

EdgeListNode_ptr NextOutNeighbor(RrtNodeNeighborIterator_ptr &iterator);
EdgeListNode_ptr NextInNeighbor(RrtNodeNeighborIterator_ptr &iterator);

#endif // NEIGHBOR_ITERATOR_H
