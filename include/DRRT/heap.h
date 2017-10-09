#ifndef HEAP_H
#define HEAP_H

#include <DRRT/libraries.h>
#include <DRRT/heapnode.h>

#define UNSET_IDX -1

// Binary heap
// Dummy node at position 0, so top of heap is at index 1
class Heap
{
    bool flag_;                          // Type of heap Max(1) or Min(0)
    std::vector<HeapNode_ptr> heap_;
    int index_of_last_;
    int parent_of_last_;

    // Min Heap Operations
    void AddMin(HeapNode_ptr &node);
    void RemoveMin(HeapNode_ptr &node);
    void UpdateMin(HeapNode_ptr &node);
    void TopMin(HeapNode_ptr &node);
    void PopMin(HeapNode_ptr &node);
    void BubbleUpMin(int idx);
    void BubbleDownMin(int idx);
    bool CheckMin();

    // Max Heap Operations
    void AddMax(HeapNode_ptr &node);
    void RemoveMax(HeapNode_ptr &node);
    void UpdateMax(HeapNode_ptr &node);
    void TopMax(HeapNode_ptr &node);
    void PopMax(HeapNode_ptr &node);
    void BubbleUpMax(int idx);
    void BubbleDownMax(int idx);
    bool CheckMax();

public:
    Heap(bool type) : flag_(type), index_of_last_(0), parent_of_last_(-1)
    { heap_.push_back(std::make_shared<HeapNode>()); }

    std::string GetType() { return (flag_ ? "Max" : "Min"); }
    int GetHeapSize() { return heap_.size() - 1; }
    bool IsEmpty() { return GetHeapSize() == 0; }
    void GetHeap(std::vector<HeapNode_ptr> &heap) { heap = heap_; }
    int GetIndexOfLast() { return index_of_last_; }
    int GetParentOfLast() { return parent_of_last_; }

    // Accessor Operations
    void Add(HeapNode_ptr &node) { return (flag_ ? AddMax(node) : AddMin(node)); }
    void Remove(HeapNode_ptr &node) { return (flag_ ? RemoveMax(node) : RemoveMin(node)); }
    void Update(HeapNode_ptr &node) { return (flag_ ? UpdateMax(node) : UpdateMin(node)); }
    void Top(HeapNode_ptr &node) { return (flag_ ? TopMax(node) : TopMin(node)); }
    void Pop(HeapNode_ptr &node) { return (flag_ ? PopMax(node) : PopMin(node)); }
    void BubbleUp(int idx) { return (flag_ ? BubbleUpMax(idx) : BubbleUpMin(idx)); }
    void BubbleDown(int idx) { return (flag_ ? BubbleDownMax(idx) : BubbleDownMin(idx)); }
    void Check() { (flag_ ? CheckMax() : CheckMin()); }

    void Clean(std::vector<HeapNode_ptr> &heap);
};

typedef std::shared_ptr<Heap> Heap_ptr;

#endif // HEAP_H
