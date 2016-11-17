/* jlist.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/jlist.h>
#include <DRRT/kdtreenode.h>

void JList::JlistPush( KDTreeNode* t )
{
    JListNode* newNode = new JListNode( t );
    newNode->parent = front->parent;
    newNode->child = front;

    if( length == 0 ) {
        back = newNode;
    } else {
        front->parent = newNode;
    }

    front = newNode;
    length += 1;
}

void JList::JlistPush( KDTreeNode* t, double k )
{
    JListNode* newNode = new JListNode( t );
    newNode->parent = front->parent;
    newNode->child = front;

    if( length == 0 ) {
        back = newNode;
    } else {
        front->parent = newNode;
    }

    newNode->key = k;
    front = newNode;
    length += 1;
}

KDTreeNode* JList::JlistTop()
{
    if( length == 0 ) {
        // Jlist is empty
        return new KDTreeNode();
    }
    return front->node;
}

void JList::JlistTopKey( KDTreeNode* n, double* k )
{
    if( length == 0 ) {
        // Jlist is empty
        n = new KDTreeNode();
        *k = -1.0;
    }
    n = front->node;
    *k = front->key;
}

KDTreeNode* JList::JlistPop()
{
    if( length == 0 ) {
        // Jlist is empty
        return new KDTreeNode();
    }

    JListNode* oldTop = front;
    if( length > 1 ) {
        front->child->parent = front->parent;
        front = front->child;
    } else if( length == 1 ) {
        back = bound;
        front = bound;
    }

    length -= 1;

    oldTop->child = oldTop; // added in case Jlist nodes hang around after this
    oldTop->parent = oldTop;

    return oldTop->node;
}

void JList::JlistPopKey( KDTreeNode* n, double* k )
{
    if( length == 0 ) {
        // Jlist is empty
        n = new KDTreeNode();
        *k = -1.0;
    }

    JListNode* oldTop = front;
    if( length > 1 ) {
        front->child->parent = front->parent;
        front = front->child;
    } else if( length == 1 ) {
        back = bound;
        front = bound;
    }

    length -= 1;

    oldTop->child = oldTop;
    oldTop->parent = oldTop;

    n = oldTop->node;
    *k = oldTop->key;
}

// Removes node from the list
bool JList::JlistRemove( JListNode* node )
{
    if( length == 0 ) {
        // Node not in Jlist
        return true;
    }

    if( front == node ) {
        front = node->child;
    }
    if( back == node ) {
        back = node->parent;
    }

    JListNode* nextNode = node->child;
    JListNode* previousNode = node->parent;

    if( length > 1 && previousNode != previousNode->child ) {
        previousNode->child = nextNode;
    }
    if( length > 1 && nextNode != nextNode->parent ) {
        nextNode->parent = previousNode;
    }

    length -= 1;

    if( length == 0 ) {
        back = bound; // dummy node
        front = bound; // dummy node
    }

    node->parent = node;
    node->child = node;

    return true;
}

void JList::JlistPrint()
{
    JListNode* ptr = front;
    while( ptr != ptr->child ) {
        std::cout << ptr->node->dist << std::endl;
        ptr = ptr->child;
    }
}

void JList::JlistEmpty()
{
    while( JlistPop()->dist != -1.0 );
}

/* Test case
int main()
{
    JList L = JList();

    KDTreeNode* a = new KDTreeNode(1);
    KDTreeNode* b = new KDTreeNode(2);
    KDTreeNode* c = new KDTreeNode(3);
    KDTreeNode* d = new KDTreeNode(4);

    std::cout << "Pushing three nodes onto list" << std::endl;
    L.JlistPush(a);
    L.JlistPush(b);
    L.JlistPush(c);

    std::cout << "Printing list" << std::endl;
    L.JlistPrint();

    std::cout << "Emptying list" << std::endl;
    L.JlistEmpty();

    std::cout << "-" << std::endl;

    std::cout << "Printing list" << std::endl;
    L.JlistPrint();

    std::cout << "-" << std::endl;

    std::cout << "Pushing five nodes onto list" << std::endl;
    L.JlistPush(a);
    L.JlistPush(b);
    L.JlistPush(c);
    L.JlistPush(b);
    L.JlistPush(a);

    JListNode* cc = L.front;

    std::cout << "Printing list" << std::endl;
    L.JlistPrint();

    std::cout << "-" << std::endl;

    std::cout << "Removing front node from list" << std::endl;
    L.JlistRemove(cc);

    std::cout << "Pushing node onto list" << std::endl;
    L.JlistPush(d);

    std::cout << "Printing list" << std::endl;
    L.JlistPrint();
    return 0;
}*/
