/* list.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/list.h>

void List::listPush( std::shared_ptr<KDTreeNode> tnode )
{
    ListNode* newNode = new ListNode( tnode );
    newNode->child = front;
    front = newNode;
    length += 1;
}

void List::listPush( std::shared_ptr<KDTreeNode> tnode, double key )
{
    ListNode* newNode = new ListNode( tnode );
    newNode->child = front;
    newNode->key = key;
    front = newNode;
    length += 1;
}

std::shared_ptr<KDTreeNode> List::listTop()
{
    if( front == front->child ) {
        // List is empty
        return std::make_shared<KDTreeNode>(-1);
    }
    return front->tnode;
}

void List::listTopKey( std::shared_ptr<KDTreeNode> n, double* k )
{
    if( front == front->child ) {
        // List is empty
        n = std::make_shared<KDTreeNode>(-1);
        *k = -1;
    }
    n = front->tnode;
    *k = front->key;
}

std::shared_ptr<KDTreeNode> List::listPop()
{
    if( front == front->child ) {
        // List is empty
        return std::make_shared<KDTreeNode>(-1);
    }
    ListNode* oldTop = front;
    front = front->child;
    length -= 1;
    return oldTop->tnode;
}

void List::listPopKey( std::shared_ptr<KDTreeNode> n, double* k )
{
    if( front == front->child ) {
        // List is empty
        n = std::make_shared<KDTreeNode>(-1);
        *k = -1;
    }
    ListNode* oldTop = front;
    front = front->child;
    length -= 1;
    n = oldTop->tnode;
    k = &oldTop->key;
}

void List::listEmpty()
{
    while( listPop()->dist != -1 );
}

void List::listPrint()
{
    ListNode* ptr = front;
    while( ptr != ptr->child ) {
        std::cout << ptr->tnode->dist << std::endl;
        ptr = ptr->child;
    }
}

List List::listCopy( ListNode* example )
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

/* Test case
int main()
{
    List L = List();

    std::shared_ptr<KDTreeNode> a = new KDTreeNode(1);
    std::shared_ptr<KDTreeNode> b = new KDTreeNode(2);
    std::shared_ptr<KDTreeNode> c = new KDTreeNode(3);

    L.listPush(a,1);
    L.listPush(b,2);
    L.listPush(c,3);

    L.listPrint();
    std::cout << "list printed" << std::endl;

    L.listEmpty();
    std::cout << "list emptied" << std::endl;

    L.listPrint();
    std::cout << "list printed" << std::endl;

    L.listPush(a,1);
    L.listPush(b,2);

    L.listPrint();
    std::cout << "list printed" << std::endl;

    std::cout << "-- copy:" << std::endl;
    List L2 = L.listCopy( L.front );
    L2.listPrint();

    std::cout << "-- original:" << std::endl;
    L.listPrint();

    std::cout << "L.length =?= L2.length" << " : " << L.length << " =?= " << L2.length << std::endl;

    L.listPush(c,3);
    L2.listPush(c,3);
    L.listPrint();
    std::cout << "list printed" << std::endl;
    L2.listPrint();
    std::cout << "list2 printed" << std::endl;
    std::cout << "L.length =?= L2.length" << " : " << L.length << " =?= " << L2.length << std::endl;
}*/
