/* list.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/list.h>

using namespace std;

void List::listPush(shared_ptr<Obstacle>& o)
{
    shared_ptr<ListNode> newNode = make_shared<ListNode>(o);
    newNode->child_ = front_;
    front_ = newNode;
    length_ += 1;
}

void List::listPush(shared_ptr<Obstacle>& o, double k)
{
    shared_ptr<ListNode> newNode = make_shared<ListNode>(o);
    newNode->child_ = front_;
    newNode->key_ = k;
    front_ = newNode;
    length_ += 1;
}

void List::listTop(shared_ptr<Obstacle>& o)
{
    if( front_ == front_->child_ ) {
        // List is empty
        o = make_shared<Obstacle>(-1);
    }
    o = front_->obstacle_;
}

void List::listTopKey(shared_ptr<Obstacle>& o, double* k)
{
    if( front_ == front_->child_ ) {
        // List is empty
        o = make_shared<Obstacle>(-1);
        *k = -1;
    }
    o = front_->obstacle_;
    *k = front_->key_;
}

void List::listPop(shared_ptr<Obstacle>& o)
{
    if( front_ == front_->child_ ) {
        // List is empty
        o = make_shared<Obstacle>(-1);
    }
    shared_ptr<ListNode> oldTop = front_;
    front_ = front_->child_;
    length_ -= 1;
    o = oldTop->obstacle_;
}

void List::listPopKey(shared_ptr<Obstacle>& o, double* k)
{
    if( front_ == front_->child_ ) {
        // List is empty
        o = make_shared<Obstacle>(-1);
        *k = -1;
    }
    shared_ptr<ListNode> oldTop = front_;
    front_ = front_->child_;
    length_ -= 1;
    o = oldTop->obstacle_;
    k = &oldTop->key_;
}

void List::listEmpty()
{
    shared_ptr<Obstacle> temp = make_shared<Obstacle>(-1);
    listPop(temp);
    while( temp->kind_ != -1 ) listPop(temp);
}

void List::listPrint()
{
    cout << "Printing Obstacles:" << endl;
    shared_ptr<ListNode> ptr = front_;
    while( ptr != ptr->child_ ) {
        cout << ptr->obstacle_->kind_ << endl;
        ptr = ptr->child_;
    }
}

shared_ptr<List> List::listCopy(shared_ptr<ListNode> example)
{
    shared_ptr<List> newList = make_shared<List>();
    shared_ptr<ListNode> ptr = front_;
    shared_ptr<ListNode> newfront_ = make_shared<ListNode>( example->obstacle_ );
    newList->front_ = newfront_;
    shared_ptr<ListNode> new_ptr = newList->front_;

    while( ptr != ptr->child_ ) {
        shared_ptr<ListNode> newNode = make_shared<ListNode>( example->obstacle_ );
        new_ptr->child_ = newNode;
        new_ptr->obstacle_ = ptr->obstacle_;
        new_ptr->key_ = ptr->key_;

        new_ptr = new_ptr->child_;
        ptr = ptr->child_;
    }
    new_ptr->child_ = new_ptr;
    new_ptr->obstacle_ = ptr->obstacle_;
    new_ptr->key_ = ptr->key_;
    newList->length_ = length_;

    return newList;
}

/* Test case
int main()
{
    List L = List();

    shared_ptr<Obstacle> a = new Obstacle(1);
    shared_ptr<Obstacle> b = new Obstacle(2);
    shared_ptr<Obstacle> c = new Obstacle(3);

    L.listPush(a,1);
    L.listPush(b,2);
    L.listPush(c,3);

    L.listPrint();
    cout << "list printed" << endl;

    L.listEmpty();
    cout << "list emptied" << endl;

    L.listPrint();
    cout << "list printed" << endl;

    L.listPush(a,1);
    L.listPush(b,2);

    L.listPrint();
    cout << "list printed" << endl;

    cout << "-- copy:" << endl;
    List L2 = L.listCopy( L.front_ );
    L2.listPrint();

    cout << "-- original:" << endl;
    L.listPrint();

    cout << "L.length_ =?= L2.length_" << " : " << L.length_ << " =?= " << L2.length_ << endl;

    L.listPush(c,3);
    L2.listPush(c,3);
    L.listPrint();
    cout << "list printed" << endl;
    L2.listPrint();
    cout << "list2 printed" << endl;
    cout << "L.length_ =?= L2.length_" << " : " << L.length_ << " =?= " << L2.length_ << endl;
}*/
