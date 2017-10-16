#ifndef LIST_H
#define LIST_H

#include <DRRT/libraries.h>

// Template class must have GetChild() and data_ member variables
// Lists always end with an empty node of type T with child equal to self

template <class T>
class List
{
    std::shared_ptr<T> front_;
    std::shared_ptr<T> back_;
    std::shared_ptr<T> bound_;
    int length_;

public:
    List();
    List(std::shared_ptr<T> front_node);

    int GetLength();
    std::shared_ptr<T> GetFront() { return front_; }
    std::shared_ptr<T> GetBack() { return back_; }
    std::shared_ptr<T> GetBound() { return bound_; }
    void Push(std::shared_ptr<T> &new_node);
    void Top(std::shared_ptr<T> &top_node);
    void Pop(std::shared_ptr<T> &top_node);
    void Remove(std::shared_ptr<T> &node);
    void Empty();
//    void Print();
};

#endif // LIST_H
