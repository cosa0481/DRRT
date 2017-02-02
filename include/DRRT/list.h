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
    std::shared_ptr<KDTreeNode> tnode;
    ListNode* child;
    double key = 0.0;

    // Constructors
    ListNode(std::shared_ptr<KDTreeNode> n) : tnode(n) {}
    ListNode(){}
};

// A simple list
class List {
public:
    ListNode* front;
    int length;

    // Constructors
    List()
    {
        ListNode* endNode = new ListNode();
        endNode->child = endNode;
        front = endNode;
        length = 0;
    }

    void listPush( std::shared_ptr<KDTreeNode> tnode );
    void listPush( std::shared_ptr<KDTreeNode> tnode, double key );
    std::shared_ptr<KDTreeNode> listTop();
    void listTopKey( std::shared_ptr<KDTreeNode> n, double* k );
    std::shared_ptr<KDTreeNode> listPop();
    void listPopKey( std::shared_ptr<KDTreeNode> n, double* k );
    void listEmpty();
    void listPrint();

    // Helper function
    List listCopy( ListNode* example );
};


#endif // LIST_H
