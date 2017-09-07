#ifndef HEAPNODE_H
#define HEAPNODE_H

#include <DRRT/libraries.h>

class HeapNode
{
    bool is_empty_;
    int index_;
    bool in_heap_;
    double value_;

public:
    HeapNode(double val) : is_empty_(false), index_(-1), in_heap_(false), value_(val) {}
    HeapNode() : is_empty_(true), index_(-1), in_heap_(false), value_(-1) {}

    virtual void GetData() {}
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetIndex(int idx) { index_ = idx; }
    int GetIndex() { return index_; }
    void SetInHeap(bool in_heap) { in_heap_ = in_heap; }
    bool InHeap() { return in_heap_; }
    void SetValue(double val) { value_ = val; }
    double GetValue() { return value_; }
};

#endif // HEAPNODE_H
