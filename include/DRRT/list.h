#ifndef LIST_H
#define LIST_H

#include <DRRT/libraries.h>
#include <DRRT/listnode.h>

// Template class must have child_ and data_ member variables
// Lists always end with an empty node of type T with child equal to self
class List
{
    std::shared_ptr<ListNode> front_;
    int length_;

public:
    List() {
        std::shared_ptr<ListNode> end = std::make_shared<ListNode>();
        end->child_ = end;
        front_ = end;
        length_ = 0;
    }

    List(std::shared_ptr<ListNode> front_node) {
        std::shared_ptr<ListNode> end = std::make_shared<ListNode>();
        end->child_ = end;
        front_ = end;
        length_ = 0;
        List::Push(front_node);
    }


    int GetLength();
    void Push(std::shared_ptr<ListNode> &new_node);
    void Top(std::shared_ptr<ListNode> &top_node);
    void Pop(std::shared_ptr<ListNode> &top_node);
    void Empty();
};

#endif // LIST_H
