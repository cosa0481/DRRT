/* heap.cpp
 * Corin Sandford
 * Test written by: Michael Otte
 * Fall 2016
 * Test for BinaryHeap and this->HeapNode classes
 */

#include <DRRT/heap.h>

void BinaryHeap::displayHeap()
{
    std::cout << "Heap Size: " << H.size() << std::endl;
    for(int i = 0; i < H.size(); i++ ) {
        std::cout << "HeapNode " << i << ": " << H[i] << std::endl;
        if( H[i]->dist_ != -1) {
            std::cout << H[i]->position << std::endl;
        } else {
            std::cout << "dummy node" << std::endl;
        }
    }
    std::cout << std::endl;
}

void BinaryHeap::getHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap)
{ heap = this->H; }

/* Heap operation functions for returning the smallest thing */

bool BinaryHeap::bubbleUp( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;
    while( n > 1 && greaterThan(this->H[parent], this->H[n]) ) {
        // Swap graph node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->H[parent];
        this->H[parent] = this->H[n];
        this->H[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->H[parent], parent );
        setIndex( this->H[n], n );

        // Get new node and parent indicies
        n = parent;
        parent = n/2;
    }
    return true;
}

bool BinaryHeap::bubbleDown( int n )
{
    int child;
    if( 2*n == indexOfLast ) {
        child = 2*n;
    } else if ( 2*n+1 > indexOfLast ) {
        return true;
    } else if ( lessThan(this->H[2*n],this->H[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parentOfLast && lessThan(this->H[child],this->H[n]) ) {
        // Swap node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->H[child];
        this->H[child] = this->H[n];
        this->H[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->H[child], child );
        setIndex( this->H[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == indexOfLast ) {
            child = 2*n;
        } else if ( 2*n+1 > indexOfLast ) {
            return true;
        } else if ( lessThan(this->H[2*n],this->H[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::addToHeap(std::shared_ptr<KDTreeNode> &node)
{
    if( !marked(node) ) {
//        std::cout << "adding: " << node << "\n" << node->position << std::endl;
        indexOfLast += 1;
        parentOfLast = indexOfLast/2;
        H.push_back( node );
        setIndex( H[indexOfLast], indexOfLast );
        mark( H[indexOfLast] );
        bubbleUp( indexOfLast );
    } else {
        // Node is already in heap
        std::cout << "node already in heap" << std::endl;
        return false;
    }
    return true;
}

void BinaryHeap::topHeap(std::shared_ptr<KDTreeNode> &node)
{
    if( indexOfLast >= 1 ) node = this->H[1];
}

void BinaryHeap::popHeap(std::shared_ptr<KDTreeNode> &node)
{
//    std::cout << "popping: " << node << "\n" << node->position << std::endl;
    std::shared_ptr<KDTreeNode> oldTopNode;
//    std::cout << "indexOfLast: " << indexOfLast << std::endl;
    if( indexOfLast > 1 ) {
        oldTopNode = this->H[1];
        this->H[1] = this->H[indexOfLast];
        this->H.erase(H.begin()+indexOfLast);
        setIndex( this->H[1], 1 );
    } else if( indexOfLast == 1 ) {
        oldTopNode = this->H[1];
        this->H.erase(H.begin()+1);
    }
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleDown( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    node = oldTopNode;
}

bool BinaryHeap::removeFromHeap( std::shared_ptr<KDTreeNode> &node )
{
    int n = getIndex(node);
    this->H[n] = this->H[indexOfLast];
    setIndex( this->H[n], n );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleUp( n );
    bubbleDown( getIndex(this->H[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::updateHeap( std::shared_ptr<KDTreeNode> &node )
{
    if( !marked( node ) ) {
        // Node not in the heap
        return false;
    }
    bubbleUp( getIndex( node ) );
    bubbleDown( getIndex( node ) );
    return true;
}

bool BinaryHeap::checkHeap()
{

    int i = 2;
    if( indexOfLast < 1 ) {
        std::cout << "this->Heap is empty" << std::endl;
        return true;
    } else if( getIndex(this->H[1]) != 1 ) {
        std::cout << "getIndex(this->H[1]) = " << getIndex(this->H[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= indexOfLast ) {
        if( lessThan(this->H[i],this->H[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(this->H[i]) != i ) {
            std::cout << "There is a problem with the heap node data: "
                      << "getIndex(this->H["<<i<<"]) != "<<i << std::endl;
            for( int j = 1; j<=indexOfLast; j++ ) {
                std::cout << "getIndex(this->H["<<j<<"]) = "
                          << getIndex(this->H[j]) << std::endl;
            }
            return false;
        }
        i += 1;
    }

    std::cout << "The heap is OK" << std::endl;
    return true;
}

void BinaryHeap::cleanHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap)
{
    for( int i = 0; i <= indexOfLast; i++ ) {
        unmark( this->H[i] );
        unsetIndex( this->H[i] );
    }

    indexOfLast = 0;
    parentOfLast = -1;

    heap = this->H;
}

/* this->Heap operation functions for returning the largest thing */

bool BinaryHeap::bubbleUpB( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;

    while( n > 1 && lessThan(this->H[parent],this->H[n]) ) {
        // Swap graph node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->H[parent];
        this->H[parent] = this->H[n];
        this->H[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->H[parent], parent );
        setIndex( this->H[n], n );

        // Get new node and parent indicies
        n = parent;
        parent = n/2;
    }
    return true;
}

bool BinaryHeap::bubbleDownB( int n )
{
    int child;
    if( 2*n == indexOfLast ) {
        child = 2*n;
    } else if ( 2*n+1 > indexOfLast ) {
        return true;
    } else if ( greaterThan(this->H[2*n],this->H[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parentOfLast && greaterThan(this->H[child],this->H[n]) ) {
        // Swap node pointers
        std::shared_ptr<KDTreeNode> tempNode = this->H[child];
        this->H[child] = this->H[n];
        this->H[n] = tempNode;

        // Update graph node heap index values
        setIndex( this->H[child], child );
        setIndex( this->H[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == indexOfLast ) {
            child = 2*n;
        } else if ( 2*n+1 > indexOfLast ) {
            return true;
        } else if ( greaterThan(this->H[2*n],this->H[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::addToHeapB( std::shared_ptr<KDTreeNode> &node )
{
    if( !marked(node) ) {
        indexOfLast += 1;
        parentOfLast = indexOfLast/2;
        this->H.push_back( node );
        setIndex( this->H[indexOfLast], indexOfLast );
        mark( this->H[indexOfLast] );
        bubbleUpB( indexOfLast );
    } else {
        // Node is already in heap
        return false;
    }
    return true;
}

void BinaryHeap::popHeapB(std::shared_ptr<KDTreeNode> &node)
{
    std::cout << "popping: " << node << "\n" << node->position << std::endl;
    std::shared_ptr<KDTreeNode> oldTopNode;
    if( indexOfLast > 1 ) {
        oldTopNode = this->H[1];
        this->H[1] = this->H[indexOfLast];
        this->H.erase(H.begin()+indexOfLast);
        setIndex( this->H[1], 1 );
    } else if( indexOfLast == 1 ) {
        oldTopNode = this->H[1];
        this->H.erase(H.begin()+1);
    }
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleDownB( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    node = oldTopNode;
}

bool BinaryHeap::removeFromHeapB( std::shared_ptr<KDTreeNode> node )
{
    int n = getIndex( node );
    this->H[n] = this->H[indexOfLast];
    setIndex( this->H[n], n );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleUpB( n );
    bubbleDownB( getIndex(this->H[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::updateHeapB( std::shared_ptr<KDTreeNode> node )
{
    if( !marked( node ) ) {
        // Node not in the heap
        return false;
    }
    bubbleUpB( getIndex( node ) );
    bubbleDownB( getIndex( node ) );
    return true;
}

bool BinaryHeap::checkHeapB()
{
    int i = 2;
    if( indexOfLast < 1 ) {
        std::cout << "this->Heap is empty" << std::endl;
        return true;
    } else if( getIndex(this->H[1]) != 1 ) {
        std::cout << getIndex(this->H[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= indexOfLast ) {
        if( greaterThan(this->H[i],this->H[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(this->H[i]) != i ) {
            std::cout << "There is a problem with the heap node data: "
                      << "getIndex(this->H["<<i<<"]) != "<< i << std::endl;
            for( int j = 1; j<=indexOfLast; j++ ) {
                std::cout << "getIndex(this->H["<<j<<"]) = "
                          << getIndex(this->H[j]) << std::endl;
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

    BinaryHeap Bthis->H = BinaryHeap();
    srand(time(NULL));
    double r;
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !Bthis->H.addTothis->Heap( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             Bthis->H.popthis->Heap();
         } else if( *Bthis->H.getIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*Bthis->H.getIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *Bthis->H.getthis->Heap();
             Bthis->H.removeFromthis->Heap( &heap[ randN ] );
         }

         Bthis->H.checkthis->Heap();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5
                && *Bthis->H.getIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*Bthis->H.getIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *Bthis->H.getthis->Heap();
             heap[ randN ].dist_ = (double) rand() / (RAND_MAX);
             if( !Bthis->H.updatethis->Heap( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN
                           << " not in heap" << std::endl;
             }
             Bthis->H.checkthis->Heap();
         }


    }

    std::cout << "--------------------------------------------" << std::endl;

    // Tests heap operation function for returning the largest thing

    Bthis->H = BinaryHeap();
    srand(time(NULL));
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !Bthis->H.addTothis->HeapB( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             Bthis->H.popthis->HeapB();
         } else if( *Bthis->H.getIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*Bthis->H.getIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *Bthis->H.getthis->Heap();
             Bthis->H.removeFromthis->HeapB( &heap[ randN ] );
         }

         Bthis->H.checkthis->HeapB();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5
                && *Bthis->H.getIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*Bthis->H.getIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *Bthis->H.getthis->Heap();
             heap[ randN ].dist_ = (double) rand() / (RAND_MAX);
             if( !Bthis->H.updatethis->HeapB( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN
                           << " not in heap" << std::endl;
             }
             Bthis->H.checkthis->HeapB();
         }


    }
    return 0;
}*/
