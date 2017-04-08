/* jlist.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/jlist.h>
#include <DRRT/kdtreenode.h>
#include <DRRT/edge.h>

bool JList::JListContains(std::shared_ptr<KDTreeNode> &t)
{
    std::shared_ptr<JListNode> ptr = front_;
    while( ptr != ptr->child_ ) {
        if( t == ptr->node_ ) {
            return true;
        }
        ptr = ptr->child_;
    }
    return false;
}

void JList::JListPush( std::shared_ptr<KDTreeNode> &t )
{
    std::shared_ptr<JListNode> newNode = std::make_shared<JListNode>( t );
    newNode->parent_ = front_->parent_;
    newNode->child_ = front_;

    if( length_ == 0 ) {
        back_ = newNode;
    } else {
        front_->parent_ = newNode;
    }

    front_ = newNode;
    length_ += 1;
}

void JList::JListPush( std::shared_ptr<Edge> &e )
{
    std::shared_ptr<JListNode> newNode = std::make_shared<JListNode>( e );
    newNode->parent_ = front_->parent_;
    newNode->child_ = front_;

    if( length_ == 0 ) {
        back_ = newNode;
    } else {
        front_->parent_ = newNode;
    }

    front_ = newNode;
    length_ += 1;
}

void JList::JListPush( std::shared_ptr<KDTreeNode> &t, double k )
{
    std::shared_ptr<JListNode> newNode = std::make_shared<JListNode>( t );
    newNode->parent_ = front_->parent_;
    newNode->child_ = front_;

    if( length_ == 0 ) {
        back_ = newNode;
    } else {
        front_->parent_ = newNode;
    }

    newNode->key_ = k;
    front_ = newNode;
    length_ += 1;
}

void JList::JListPush( std::shared_ptr<Edge> &e, double k )
{
    std::shared_ptr<JListNode> newNode = std::make_shared<JListNode>( e );
    newNode->parent_ = front_->parent_;
    newNode->child_ = front_;

    if( length_ == 0 ) {
        back_ = newNode;
    } else {
        front_->parent_ = newNode;
    }

    newNode->key_ = k;
    front_ = newNode;
    length_ += 1;
}

void JList::JListTop( std::shared_ptr<KDTreeNode> &t )
{
    if( length_ == 0 ) {
        // Jlist is empty
        t = std::make_shared<KDTreeNode>();
    } else {
        t = front_->node_;
    }
}

void JList::JListTop( std::shared_ptr<Edge> &e ) {
    if( length_ == 0 ) {
        // Jlist is empty
        e->dist_ = -1;
    } else {
        e = front_->edge_;
    }
}

void JList::JListTopKey( std::shared_ptr<KDTreeNode> &t,
                         std::shared_ptr<double> k )
{
    if( length_ == 0 ) {
        // Jlist is empty
        t = std::make_shared<KDTreeNode>();
        *k = -1.0;
    } else {
        t = front_->node_;
        *k = front_->key_;
    }
}

void JList::JListTopKey( std::shared_ptr<Edge> &e, std::shared_ptr<double> k )
{
    if( length_ == 0 ) {
        // Jlist is empty
        e->dist_ = -1;
        *k = -1.0;
    } else {
        e = front_->edge_;
        *k = front_->key_;
    }
}

void JList::JListPop( std::shared_ptr<KDTreeNode> &t )
{
    if( length_ == 0 ) {
        // Jlist is empty
        t = std::make_shared<KDTreeNode>();
    } else {
        std::shared_ptr<JListNode> oldTop = front_;
        if( length_ > 1 ) {
            front_->child_->parent_ = front_->parent_;
            front_ = front_->child_;
        } else if( length_ == 1 ) {
            back_ = bound_;
            front_ = bound_;
        }

        length_ -= 1;

        oldTop->child_ = oldTop; // added in case Jlist nodes hang around after this
        oldTop->parent_ = oldTop;

        t = oldTop->node_;
    }
}

void JList::JListPop( std::shared_ptr<Edge> &e )
{
    if( length_ == 0 ) {
        // Jlist is empty
        e->dist_ = -1;
    } else {
        std::shared_ptr<JListNode> oldTop = front_;
        if( length_ > 1 ) {
            front_->child_->parent_ = front_->parent_;
            front_ = front_->child_;
        } else if( length_ == 1 ) {
            back_ = bound_;
            front_ = bound_;
        }

        length_ -= 1;

        oldTop->child_ = oldTop; // added in case Jlist nodes hang around after this
        oldTop->parent_ = oldTop;

        e = oldTop->edge_;
    }
}

void JList::JListPopKey( std::shared_ptr<KDTreeNode> &n,
                         std::shared_ptr<double> k)
{
    if( length_ == 0 ) {
        // Jlist is empty
        n = std::make_shared<KDTreeNode>();
        *k = -1.0;
    } else {
        std::shared_ptr<JListNode> oldTop = front_;
        if( length_ > 1 ) {
            front_->child_->parent_ = front_->parent_;
            front_ = front_->child_;
        } else if( length_ == 1 ) {
            back_ = bound_;
            front_ = bound_;
        }

        length_ -= 1;

        oldTop->child_ = oldTop;
        oldTop->parent_ = oldTop;

        n = oldTop->node_;
        *k = oldTop->key_;
    }
}

void JList::JListPopKey(std::shared_ptr<Edge> &e, std::shared_ptr<double> k)
{
    if( length_ == 0 ) {
        // Jlist is empty
        e->dist_ = -1;
        *k = -1.0;
    } else {
        std::shared_ptr<JListNode> oldTop = front_;
        if( length_ > 1 ) {
            front_->child_->parent_ = front_->parent_;
            front_ = front_->child_;
        } else if( length_ == 1 ) {
            back_ = bound_;
            front_ = bound_;
        }

        length_ -= 1;

        oldTop->child_ = oldTop;
        oldTop->parent_ = oldTop;

        e = oldTop->edge_;
        *k = oldTop->key_;
    }
}

// Removes node_ from the list
bool JList::JListRemove(std::shared_ptr<JListNode> &node )
{
    if( length_ == 0 ) {
        // Node not in Jlist
        return true;
    }

    if( front_ == node ) {
        front_ = node->child_;
    }
    if( back_ == node ) {
        back_ = node->parent_;
    }

    std::shared_ptr<JListNode> nextNode = node->child_;
    std::shared_ptr<JListNode> previousNode = node->parent_;

    if( length_ > 1 && previousNode != previousNode->child_ ) {
        previousNode->child_ = nextNode;
    }
    if( length_ > 1 && nextNode != nextNode->parent_ ) {
        nextNode->parent_ = previousNode;
    }

    length_ -= 1;

    if( length_ == 0 ) {
        back_ = bound_; // dummy node_
        front_ = bound_; // dummy node_
    }

    node->parent_ = node;
    node->child_ = node;

    return true;
}

void JList::JListPrint()
{
    if( this->length_ == 0 ) std::cout << "NOTHING";
    std::cout << std::endl;
    std::shared_ptr<JListNode> ptr = front_;
    int i = 1;
    while( ptr != ptr->child_ ) {
        if( use_nodes_ ) {
            std::cout << "node_ " << i << ": "
                      << ptr->node_->rrt_LMC_ << "\n" << ptr->node_->position_
                      << std::endl;
        } else { // use edges
            std::cout << "edge_ " << i << ": "
                      << ptr->edge_->dist_ << "\n"
                      << ptr->edge_->start_node_->position_ << "\n->\n"
                      << ptr->edge_->end_node_->position_ << std::endl;
        }
        ptr = ptr->child_;
        i++;
    }
}

Eigen::Matrix<double,Eigen::Dynamic,2> JList::JListAsMatrix()
{
    Eigen::Matrix<double,Eigen::Dynamic,2> matrix(length_,2);
    std::shared_ptr<JListNode> ptr = front_;
    int row_count = 0;
    while( ptr != ptr->child_ ) {
        matrix.row(row_count) = ptr->node_->position_.head(2);
        row_count++;
        ptr = ptr->child_;
    }
    return matrix;
}

void JList::JListEmpty()
{
    std::shared_ptr<KDTreeNode> temp
            = std::make_shared<KDTreeNode>();
    JListPop(temp);
    while( temp->dist_ != -1.0 ) JListPop(temp);
}

/* Test case
int main()
{
    JList L = JList();

    std::shared_ptr<KDTreeNode> a = new KDTreeNode(1);
    std::shared_ptr<KDTreeNode> b = new KDTreeNode(2);
    std::shared_ptr<KDTreeNode> c = new KDTreeNode(3);
    std::shared_ptr<KDTreeNode> d = new KDTreeNode(4);

    std::cout << "Pushing three nodes onto list" << std::endl;
    L.JListPush(a);
    L.JListPush(b);
    L.JListPush(c);

    std::cout << "Printing list" << std::endl;
    L.JListPrint();

    std::cout << "Emptying list" << std::endl;
    L.JListEmpty();

    std::cout << "-" << std::endl;

    std::cout << "Printing list" << std::endl;
    L.JListPrint();

    std::cout << "-" << std::endl;

    std::cout << "Pushing five nodes onto list" << std::endl;
    L.JListPush(a);
    L.JListPush(b);
    L.JListPush(c);
    L.JListPush(b);
    L.JListPush(a);

    std::shared_ptr<JListNode> cc = L.front_;

    std::cout << "Printing list" << std::endl;
    L.JListPrint();

    std::cout << "-" << std::endl;

    std::cout << "Removing front_ node_ from list" << std::endl;
    L.JListRemove(cc);

    std::cout << "Pushing node_ onto list" << std::endl;
    L.JListPush(d);

    std::cout << "Printing list" << std::endl;
    L.JListPrint();
    return 0;
}*/
