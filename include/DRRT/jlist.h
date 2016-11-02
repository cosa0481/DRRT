/* jlist.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef JLIST_H
#define JLIST_H

#include <iostream>

class KDTreeNode;

// A JList node (not that key is unused for JList operations,
// but it is often helpful to have a key value associated
// with data)
class JListNode{
public:
    JListNode* child;
    JListNode* parent;
    KDTreeNode* node;
    float key = 0;

    // Corstructor
    JListNode(){}
    JListNode(KDTreeNode* t) : node(t) {}
};

// A simple JList
class JList{
public:
    JListNode* front;
    JListNode* back;
    JListNode* bound; // bounds either side of the list
    int length;

    // Constructor
    JList()
    {
        JListNode* endNode = new JListNode();
        endNode->child = endNode;
        endNode->parent = endNode;

        front = endNode;
        back = endNode;
        bound = endNode;
        length = 0;
    }

    // Functions
    void JlistPush( KDTreeNode* t );
    void JlistPush( KDTreeNode* t, float k );
    KDTreeNode* JlistTop();
    void JlistTopKey( KDTreeNode* n, float* k );
    KDTreeNode* JlistPop();
    void JlistPopKey( KDTreeNode* n, float* k );
    bool JlistRemove( JListNode* node );
    void JlistPrint();
    void JlistEmpty();
};

#endif // JLIST_H
