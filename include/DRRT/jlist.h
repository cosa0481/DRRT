/* jlist.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef JLIST_H
#define JLIST_H

#include <iostream>
#include <DRRT/edge.h>

class KDTreeNode;

// A JList node (not that key is unused for JList operations,
// but it is often helpful to have a key value associated
// with data)
class JListNode{
public:
    JListNode* child;
    JListNode* parent;
    KDTreeNode* node;
    Edge* edge;
    double key = 0.0;

    // Corstructor
    JListNode() : key(-1.0) {}
    JListNode(KDTreeNode* t) : node(t) {}
    JListNode(Edge* e) : edge(e) {}
};

// A simple JList
class JList{
public:
    JListNode* front;
    JListNode* back;
    JListNode* bound;   // bounds either side of the list
    int length;
    bool useNodes;      // flag for indicating whether this JList
                        // is one of Edges or KDTreeNode's. True
                        // if JList uses KDTreeNode's

    // Constructor
    JList( bool nodeflag ) : useNodes(nodeflag)
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
    void JlistPush( Edge* e );
    void JlistPush( KDTreeNode* t, double k );
    void JlistPush( Edge* e, double k );
    void JlistTop( KDTreeNode* t );
    void JlistTop( Edge* e );
    void JlistTopKey( KDTreeNode* n, double &k );
    void JlistTopKey( Edge* e, double &k );
    void JlistPop( KDTreeNode* t );
    void JlistPop( Edge* e );
    void JlistPopKey( KDTreeNode* n, double &k );
    void JlistPopKey( Edge* e, double &k );
    bool JlistRemove( JListNode* node );
    void JlistPrint();
    void JlistEmpty();
};

#endif // JLIST_H
