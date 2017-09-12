#include <DRRT/neighbor_iterator.h>

EdgeListNode_ptr NextOutNeighbor(RrtNodeNeighborIterator_ptr &iterator)
{
    if(iterator->list_flag == 0) {
        iterator->current_item = iterator->node->GetInitialOutNeighbors()->GetFront();
        iterator->list_flag = 1;
    } else {
        iterator->current_item = iterator->current_item->GetChild();
    }
    while(iterator->current_item == iterator->current_item->GetChild()) {
        if(iterator->list_flag == 1) {
            iterator->current_item = iterator->node->GetOutNeighbors()->GetFront();
        } else {
            return std::make_shared<EdgeListNode>();
        }
        iterator->list_flag++;
    }
    return iterator->current_item;
}

EdgeListNode_ptr NextInNeighbor(RrtNodeNeighborIterator_ptr &iterator)
{
    if(iterator->list_flag == 0) {
        iterator->current_item = iterator->node->GetInitialInNeighbors()->GetFront();
        iterator->list_flag = 1;
    } else {
        iterator->current_item = iterator->current_item->GetChild();
    }
    while(iterator->current_item == iterator->current_item->GetChild()) {
        if(iterator->list_flag == 1) {
            iterator->current_item = iterator->node->GetInNeighbors()->GetFront();
        } else {
            return std::make_shared<EdgeListNode>();
        }
        iterator->list_flag++;
    }
    return iterator->current_item;
}
