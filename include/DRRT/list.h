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
    KDTreeNode* tnode;
    ListNode* child;
    float key = 0;

    // Constructors
    ListNode(KDTreeNode* n) : tnode(n) {}
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

    void listPush( KDTreeNode* tnode )
    {
        ListNode* newNode = new ListNode( tnode );
        newNode->child = front;
        front = newNode;
        length += 1;
    }

    void listPush( KDTreeNode* tnode, float key )
    {
        ListNode* newNode = new ListNode( tnode );
        newNode->child = front;
        newNode->key = key;
        front = newNode;
        length += 1;
    }

    KDTreeNode* listTop()
    {
        if( front == front->child ) {
            // List is empty
            return new KDTreeNode(-1);
        }
        return front->tnode;
    }

    void listTopKey( KDTreeNode* n, float* k )
    {
        if( front == front->child ) {
            // List is empty
            n = new KDTreeNode(-1);
            *k = -1;
        }
        n = front->tnode;
        *k = front->key;
    }

    KDTreeNode* listPop()
    {
        if( front == front->child ) {
            // List is empty
            return new KDTreeNode(-1);
        }
        ListNode* oldTop = front;
        front = front->child;
        length -= 1;
        return oldTop->tnode;
    }

    void listPopKey( KDTreeNode* n, float* k )
    {
        if( front == front->child ) {
            // List is empty
            n = new KDTreeNode(-1);
            *k = -1;
        }
        ListNode* oldTop = front;
        front = front->child;
        length -= 1;
        n = oldTop->tnode;
        k = &oldTop->key;
    }

    void listEmpty()
    {
        while( listPop()->dist != -1 );
    }

    void listPrint() {
        ListNode* ptr = front;
        while( ptr != ptr->child ) {
            std::cout << ptr->tnode->dist << std::endl;
            ptr = ptr->child;
        }
    }

    // Helper function
    List listCopy( ListNode* example )
    {
        List newList = List();
        ListNode* ptr = front;
        ListNode* newfront = new ListNode( example->tnode );
        newList.front = newfront;
        ListNode* new_ptr = newList.front;

        while( ptr != ptr->child ) {
            ListNode* newNode = new ListNode( example->tnode );
            new_ptr->child = newNode;
            new_ptr->tnode = ptr->tnode;
            new_ptr->key = ptr->key;

            new_ptr = new_ptr->child;
            ptr = ptr->child;
        }
        new_ptr->child = new_ptr;
        new_ptr->tnode = ptr->tnode;
        new_ptr->key = ptr->key;
        newList.length = length;

        return newList;
    }
};


#endif // LIST_H
