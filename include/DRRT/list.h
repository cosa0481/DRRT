/* list.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef LIST_H
#define LIST_H

#include <DRRT/kdtreenode.h>
#include <iostream>

// A list node (note that key is unused for list
// operations, but it is often helpful to have a key value
// associated with data)
class ListNode {
public:
    std::shared_ptr<Obstacle> obstacle_;
    std::shared_ptr<ListNode> child_;
    double key_ = 0.0;

    // Constructors
    ListNode(std::shared_ptr<Obstacle>& o) : obstacle_(o) {}
    ListNode() : key_(-1.0) {}
};

// A simple list
class List {
public:
    std::shared_ptr<ListNode> front_;
    int length_;

    // Constructors
    List()
    {
        std::shared_ptr<ListNode> endNode = std::make_shared<ListNode>();
        endNode->child_ = endNode;
        front_ = endNode;
        length_ = 0;
    }

    void listPush(std::shared_ptr<Obstacle>& o);
    void listPush(std::shared_ptr<Obstacle>& o, double key);
    void listTop(std::shared_ptr<Obstacle>& o);
    void listTopKey(std::shared_ptr<Obstacle>& o, double* k);
    void listPop(std::shared_ptr<Obstacle>& o);
    void listPopKey(std::shared_ptr<Obstacle>& o, double* k);
    void listEmpty();
    void listPrint();

    // Helper function
    std::shared_ptr<List> listCopy( std::shared_ptr<ListNode> example );
};


#endif // LIST_H
