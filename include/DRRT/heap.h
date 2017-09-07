#ifndef HEAP_H
#define HEAP_H

#include <DRRT/libraries.h>
#include <DRRT/heapnode.h>

#define UNSET_IDX -1

typedef std::shared_ptr<HeapNode> Hnode_ptr;

// Binary heap
// Dummy node at position 0, so top of heap is at index 1
class Heap
{
    std::vector<Hnode_ptr> heap_;
    int index_of_last_;
    int parent_of_last_;

public:
    Heap() : index_of_last_(0), parent_of_last_(-1)
    { heap_.push_back(std::make_shared<HeapNode>()); }

    int GetHeapSize() { return heap_.size() - 1; }
    bool IsEmpty() { return GetHeapSize() == 0; }
    void GetHeap(std::vector<Hnode_ptr> &heap) { heap = heap_; }
    int GetIndexOfLast() { return index_of_last_; }
    int GetParentOfLast() { return parent_of_last_; }

    // Min Heap Operations
    void AddMin(Hnode_ptr &node);
    void RemoveMin(Hnode_ptr &node);
    void UpdateMin(Hnode_ptr &node);
    void TopMin(Hnode_ptr &node);
    void PopMin(Hnode_ptr &node);
    void BubbleUpMin(int idx);
    void BubbleDownMin(int idx);
    bool CheckMin();

    // Max Heap Operations
    void AddMax(Hnode_ptr &node);
    void RemoveMax(Hnode_ptr &node);
    void UpdateMax(Hnode_ptr &node);
    void TopMax(Hnode_ptr &node);
    void PopMax(Hnode_ptr &node);
    void BubbleUpMax(int idx);
    void BubbleDownMax(int idx);
    bool CheckMax();

    void Clean(std::vector<Hnode_ptr> &heap);
};

#endif // HEAP_H
