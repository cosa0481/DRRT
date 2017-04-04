/* heap.cpp
 * Corin Sandford
 * Test written by: Michael Otte
 * Fall 2016
 * Test for BinaryHeap and this->HeapNode classes
 */

#include <DRRT/heap.h>

void BinaryHeap::DisplayHeap()
{
    std::cout << "Heap Size: " << heap_.size() << std::endl;
    for(int i = 0; i < heap_.size(); i++ ) {
        std::cout << "HeapNode " << i << ": " << heap_[i] << std::endl;
        if( heap_[i]->dist_ != -1) {
            std::cout << heap_[i]->position_ << std::endl;
        } else {
            std::cout << "dummy node" << std::endl;
        }
    }
    std::cout << std::endl;
}

void BinaryHeap::GetHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap)
{ heap = this->heap_; }

/* Heap operation functions for returning the smallest thing */

bool BinaryHeap::BubbleUp( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;
    while( n > 1 && greaterThan(this->heap_[parent], this->heap_[n]) ) {
        // Swap graph node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->heap_[parent];
        this->heap_[parent] = this->heap_[n];
        this->heap_[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->heap_[parent], parent );
        setIndex( this->heap_[n], n );

        // Get new node and parent indicies
        n = parent;
        parent = n/2;
    }
    return true;
}

bool BinaryHeap::BubbleDown( int n )
{
    int child;
    if( 2*n == index_of_last_ ) {
        child = 2*n;
    } else if ( 2*n+1 > index_of_last_ ) {
        return true;
    } else if ( lessThan(this->heap_[2*n],this->heap_[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parent_of_last_ && lessThan(this->heap_[child],this->heap_[n]) ) {
        // Swap node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->heap_[child];
        this->heap_[child] = this->heap_[n];
        this->heap_[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->heap_[child], child );
        setIndex( this->heap_[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == index_of_last_ ) {
            child = 2*n;
        } else if ( 2*n+1 > index_of_last_ ) {
            return true;
        } else if ( lessThan(this->heap_[2*n],this->heap_[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::AddToHeap(std::shared_ptr<KDTreeNode> &node)
{
    if( !marked(node) ) {
//        std::cout << "adding: " << node << "\n" << node->position_ << std::endl;
        index_of_last_ += 1;
        parent_of_last_ = index_of_last_/2;
        heap_.push_back( node );
        setIndex( heap_[index_of_last_], index_of_last_ );
        mark( heap_[index_of_last_] );
        BubbleUp( index_of_last_ );
    } else {
        // Node is already in heap
        std::cout << "node already in heap" << std::endl;
        return false;
    }
    return true;
}

void BinaryHeap::TopHeap(std::shared_ptr<KDTreeNode> &node)
{
    if( index_of_last_ >= 1 ) node = this->heap_[1];
}

void BinaryHeap::PopHeap(std::shared_ptr<KDTreeNode> &node)
{
//    std::cout << "popping: " << node << "\n" << node->position_ << std::endl;
    std::shared_ptr<KDTreeNode> oldTopNode;
//    std::cout << "index_of_last_: " << index_of_last_ << std::endl;
    if( index_of_last_ > 1 ) {
        oldTopNode = this->heap_[1];
        this->heap_[1] = this->heap_[index_of_last_];
        this->heap_.erase(heap_.begin()+index_of_last_);
        setIndex( this->heap_[1], 1 );
    } else if( index_of_last_ == 1 ) {
        oldTopNode = this->heap_[1];
        this->heap_.erase(heap_.begin()+1);
    }
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;
    BubbleDown( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    node = oldTopNode;
}

bool BinaryHeap::RemoveFromHeap( std::shared_ptr<KDTreeNode> &node )
{
    int n = getIndex(node);
    this->heap_[n] = this->heap_[index_of_last_];
    setIndex( this->heap_[n], n );
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;
    BubbleUp( n );
    BubbleDown( getIndex(this->heap_[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::UpdateHeap( std::shared_ptr<KDTreeNode> &node )
{
    if( !marked( node ) ) {
        // Node not in the heap
        return false;
    }
    BubbleUp( getIndex( node ) );
    BubbleDown( getIndex( node ) );
    return true;
}

bool BinaryHeap::CheckHeap()
{

    int i = 2;
    if( index_of_last_ < 1 ) {
        std::cout << "this->Heap is empty" << std::endl;
        return true;
    } else if( getIndex(this->heap_[1]) != 1 ) {
        std::cout << "getIndex(this->heap_[1]) = " << getIndex(this->heap_[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= index_of_last_ ) {
        if( lessThan(this->heap_[i],this->heap_[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(this->heap_[i]) != i ) {
            std::cout << "There is a problem with the heap node data: "
                      << "getIndex(this->heap_["<<i<<"]) != "<<i << std::endl;
            for( int j = 1; j<=index_of_last_; j++ ) {
                std::cout << "getIndex(this->heap_["<<j<<"]) = "
                          << getIndex(this->heap_[j]) << std::endl;
            }
            return false;
        }
        i += 1;
    }

    std::cout << "The heap is OK" << std::endl;
    return true;
}

void BinaryHeap::CleanHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap)
{
    for( int i = 0; i <= index_of_last_; i++ ) {
        unmark( this->heap_[i] );
        unsetIndex( this->heap_[i] );
    }

    index_of_last_ = 0;
    parent_of_last_ = -1;

    heap = this->heap_;
}

/* this->Heap operation functions for returning the largest thing */

bool BinaryHeap::BubbleUpB( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;

    while( n > 1 && lessThan(this->heap_[parent],this->heap_[n]) ) {
        // Swap graph node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->heap_[parent];
        this->heap_[parent] = this->heap_[n];
        this->heap_[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->heap_[parent], parent );
        setIndex( this->heap_[n], n );

        // Get new node and parent indicies
        n = parent;
        parent = n/2;
    }
    return true;
}

bool BinaryHeap::BubbleDownB( int n )
{
    int child;
    if( 2*n == index_of_last_ ) {
        child = 2*n;
    } else if ( 2*n+1 > index_of_last_ ) {
        return true;
    } else if ( greaterThan(this->heap_[2*n],this->heap_[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parent_of_last_ && greaterThan(this->heap_[child],this->heap_[n]) ) {
        // Swap node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->heap_[child];
        this->heap_[child] = this->heap_[n];
        this->heap_[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->heap_[child], child );
        setIndex( this->heap_[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == index_of_last_ ) {
            child = 2*n;
        } else if ( 2*n+1 > index_of_last_ ) {
            return true;
        } else if ( greaterThan(this->heap_[2*n],this->heap_[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::AddToHeapB( std::shared_ptr<KDTreeNode> &node )
{
    if( !marked(node) ) {
        index_of_last_ += 1;
        parent_of_last_ = index_of_last_/2;
        this->heap_.push_back( node );
        setIndex( this->heap_[index_of_last_], index_of_last_ );
        mark( this->heap_[index_of_last_] );
        BubbleUpB( index_of_last_ );
    } else {
        // Node is already in heap
        return false;
    }
    return true;
}

void BinaryHeap::PopHeapB(std::shared_ptr<KDTreeNode> &node)
{
    std::cout << "popping: " << node << "\n" << node->position_ << std::endl;
    std::shared_ptr<KDTreeNode> oldTopNode;
    if( index_of_last_ > 1 ) {
        oldTopNode = this->heap_[1];
        this->heap_[1] = this->heap_[index_of_last_];
        this->heap_.erase(heap_.begin()+index_of_last_);
        setIndex( this->heap_[1], 1 );
    } else if( index_of_last_ == 1 ) {
        oldTopNode = this->heap_[1];
        this->heap_.erase(heap_.begin()+1);
    }
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;
    BubbleDownB( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    node = oldTopNode;
}

bool BinaryHeap::RemoveFromHeapB( std::shared_ptr<KDTreeNode> node )
{
    int n = getIndex( node );
    this->heap_[n] = this->heap_[index_of_last_];
    setIndex( this->heap_[n], n );
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;
    BubbleUpB( n );
    BubbleDownB( getIndex(this->heap_[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::UpdateHeapB( std::shared_ptr<KDTreeNode> node )
{
    if( !marked( node ) ) {
        // Node not in the heap
        return false;
    }
    BubbleUpB( getIndex( node ) );
    BubbleDownB( getIndex( node ) );
    return true;
}

bool BinaryHeap::CheckHeapB()
{
    int i = 2;
    if( index_of_last_ < 1 ) {
        std::cout << "this->Heap is empty" << std::endl;
        return true;
    } else if( getIndex(this->heap_[1]) != 1 ) {
        std::cout << getIndex(this->heap_[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= index_of_last_ ) {
        if( greaterThan(this->heap_[i],this->heap_[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(this->heap_[i]) != i ) {
            std::cout << "There is a problem with the heap node data: "
                      << "getIndex(this->heap_["<<i<<"]) != "<< i << std::endl;
            for( int j = 1; j<=index_of_last_; j++ ) {
                std::cout << "getIndex(this->heap_["<<j<<"]) = "
                          << getIndex(this->heap_[j]) << std::endl;
            }
            return false;
        }
        i += 1;
    }

    std::cout << "The heap is OK" << std::endl;
    return true;
}

/* Test case
int main()
{
    std::cout << "Test of BinaryHeap and this->HeapNode data classes" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Tests heap operation functions for returning the smallest thing

    BinaryHeap Bthis->heap_ = BinaryHeap();
    srand(time(NULL));
    double r;
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !Bthis->heap_.addTothis->Heap( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             Bthis->heap_.popthis->Heap();
         } else if( *Bthis->heap_.GetIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*Bthis->heap_.GetIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *Bthis->heap_.getthis->Heap();
             Bthis->heap_.removeFromthis->Heap( &heap[ randN ] );
         }

         Bthis->heap_.checkthis->Heap();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5
                && *Bthis->heap_.GetIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*Bthis->heap_.GetIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *Bthis->heap_.getthis->Heap();
             heap[ randN ].dist_ = (double) rand() / (RAND_MAX);
             if( !Bthis->heap_.updatethis->Heap( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN
                           << " not in heap" << std::endl;
             }
             Bthis->heap_.checkthis->Heap();
         }


    }

    std::cout << "--------------------------------------------" << std::endl;

    // Tests heap operation function for returning the largest thing

    Bthis->heap_ = BinaryHeap();
    srand(time(NULL));
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !Bthis->heap_.addTothis->HeapB( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             Bthis->heap_.popthis->HeapB();
         } else if( *Bthis->heap_.GetIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*Bthis->heap_.GetIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *Bthis->heap_.getthis->Heap();
             Bthis->heap_.removeFromthis->HeapB( &heap[ randN ] );
         }

         Bthis->heap_.checkthis->HeapB();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5
                && *Bthis->heap_.GetIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*Bthis->heap_.GetIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *Bthis->heap_.getthis->Heap();
             heap[ randN ].dist_ = (double) rand() / (RAND_MAX);
             if( !Bthis->heap_.updatethis->HeapB( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN
                           << " not in heap" << std::endl;
             }
             Bthis->heap_.checkthis->HeapB();
         }


    }
    return 0;
}*/
