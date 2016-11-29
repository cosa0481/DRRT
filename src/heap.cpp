/* heap.cpp
 * Corin Sandford
 * Test written by: Michael Otte
 * Fall 2016
 * Test for BinaryHeap and HeapNode classes
 */

#include <DRRT/heap.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>

/* Heap operation functions for returning the smallest thing */

bool BinaryHeap::bubbleUp( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;
    while( n > 1 && greaterThan(H[parent], H[n]) ) {
        // Swap graph node pointers
        KDTreeNode tempNode = H[parent];
        H[parent] = H[n];
        H[n] = tempNode;

        // Update graph node heap index values
        setIndex( &H[parent], parent );
        setIndex( &H[n], n );

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
    } else if ( lessThan(H[2*n],H[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parentOfLast && lessThan(H[child],H[n]) ) {
        // Swap node pointers
        KDTreeNode tempNode = H[child];
        H[child] = H[n];
        H[n] = tempNode;

        // Update graph node heap index values
        setIndex( &H[child], child );
        setIndex( &H[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == indexOfLast ) {
            child = 2*n;
        } else if ( 2*n+1 > indexOfLast ) {
            return true;
        } else if ( lessThan(H[2*n],H[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::addToHeap( KDTreeNode* node )
{

    if( !marked(*node) ) {
        indexOfLast += 1;
        parentOfLast = indexOfLast/2;
        H.push_back( *node );
        setIndex( &H[indexOfLast], indexOfLast );
        mark( &H[indexOfLast] );
        bubbleUp( indexOfLast );
    } else {
        // Node is already in heap
        return false;
    }
    return true;
}

KDTreeNode* BinaryHeap::topHeap()
{
    if( indexOfLast < 1 ) return new KDTreeNode();
    return &H[1];
}

KDTreeNode* BinaryHeap::popHeap()
{
    if( indexOfLast < 1 ) return new KDTreeNode();
    KDTreeNode* oldTopNode = &H[1];
    H[1] = H[indexOfLast];
    setIndex( &H[1], 1 );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleDown( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    return oldTopNode;
}

bool BinaryHeap::removeFromHeap( KDTreeNode* node )
{
    int n = getIndex(*node);
    H[n] = H[indexOfLast];
    setIndex( &H[n], n );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleUp( n );
    bubbleDown( getIndex(H[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::updateHeap( KDTreeNode node )
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
        std::cout << "Heap is empty" << std::endl;
        return true;
    } else if( getIndex(H[1]) != 1 ) {
        std::cout << "getIndex(H[1]) = " << getIndex(H[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= indexOfLast ) {
        if( lessThan(H[i],H[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(H[i]) != i ) {
            std::cout << "There is a problem with the heap node data: " << "getIndex(H["<<i<<"]) != "<<i << std::endl;
            for( int j = 1; j<=indexOfLast; j++ ) {
                std::cout << "getIndex(H["<<j<<"]) = " << getIndex(H[j]) << std::endl;
            }
            return false;
        }
        i += 1;
    }

    std::cout << "The heap is OK" << std::endl;
    return true;
}

std::vector<KDTreeNode> BinaryHeap::cleanHeap()
{
    std::vector<KDTreeNode> heap = H;
    for( int i = 1; i <= indexOfLast; i++ ) {
        unmark( &H[i] );
        unsetIndex( &H[i] );
    }

    indexOfLast = 0;
    parentOfLast = -1;

    return heap;
}

/* Heap operation functions for returning the largest thing */

bool BinaryHeap::bubbleUpB( int n )
{
    if( n == 1 ) return true;

    int parent = n/2;

    while( n > 1 && lessThan(H[parent],H[n]) ) {
        // Swap graph node pointers
        KDTreeNode tempNode = H[parent];
        H[parent] = H[n];
        H[n] = tempNode;

        // Update graph node heap index values
        setIndex( &H[parent], parent );
        setIndex( &H[n], n );

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
    } else if ( greaterThan(H[2*n],H[2*n+1]) ) {
        child = 2*n;
    } else {
        child = 2*n+1;
    }

    while( n <= parentOfLast && greaterThan(H[child],H[n]) ) {
        // Swap node pointers
        KDTreeNode tempNode = H[child];
        H[child] = H[n];
        H[n] = tempNode;

        // Update graph node heap index values
        setIndex( &H[child], child );
        setIndex( &H[n], n );

        // Get new node and child indicies
        n = child;
        if( 2*n == indexOfLast ) {
            child = 2*n;
        } else if ( 2*n+1 > indexOfLast ) {
            return true;
        } else if ( greaterThan(H[2*n],H[2*n+1]) ) {
            child = 2*n;
        } else {
            child = 2*n+1;
        }
    }
    return true;
}

bool BinaryHeap::addToHeapB( KDTreeNode* node )
{
    if( !marked(*node) ) {
        indexOfLast += 1;
        parentOfLast = indexOfLast/2;
        H.push_back( *node );
        setIndex( &H[indexOfLast], indexOfLast );
        mark( &H[indexOfLast] );
        bubbleUpB( indexOfLast );
    } else {
        // Node is already in heap
        return false;
    }
    return true;
}

KDTreeNode* BinaryHeap::popHeapB()
{
    if( indexOfLast < 1 ) return new KDTreeNode();
    KDTreeNode* oldTopNode = &H[1];
    H[1] = H[indexOfLast];
    setIndex( &H[1], 1 );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleDownB( 1 );
    unmark( oldTopNode );
    unsetIndex( oldTopNode );
    return oldTopNode;
}

bool BinaryHeap::removeFromHeapB( KDTreeNode* node )
{
    int n = getIndex( *node );
    H[n] = H[indexOfLast];
    setIndex( &H[n], n );
    indexOfLast -= 1;
    parentOfLast = indexOfLast/2;
    bubbleUpB( n );
    bubbleDownB( getIndex(H[n]) );
    unmark( node );
    unsetIndex( node );
    return true;
}

bool BinaryHeap::updateHeapB( KDTreeNode node )
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
        std::cout << "Heap is empty" << std::endl;
        return true;
    } else if( getIndex(H[1]) != 1 ) {
        std::cout << getIndex(H[1]) << std::endl;
        std::cout << "There is a problem with the heap (root)" << std::endl;
        return false;
    }

    while( i <= indexOfLast ) {
        if( greaterThan(H[i],H[i/2]) ) {
            std::cout << "There is a problem with the heap order" << std::endl;
            return false;
        } else if( getIndex(H[i]) != i ) {
            std::cout << "There is a problem with the heap node data: " << "getIndex(H["<<i<<"]) != "<<i << std::endl;
            for( int j = 1; j<=indexOfLast; j++ ) {
                std::cout << "getIndex(H["<<j<<"]) = " << getIndex(H[j]) << std::endl;
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
    std::cout << "Test of BinaryHeap and HeapNode data classes" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Tests heap operation functions for returning the smallest thing

    BinaryHeap BH = BinaryHeap();
    srand(time(NULL));
    double r;
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !BH.addToHeap( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             BH.popHeap();
         } else if( *BH.getIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*BH.getIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *BH.getHeap();
             BH.removeFromHeap( &heap[ randN ] );
         }

         BH.checkHeap();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5 && *BH.getIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*BH.getIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *BH.getHeap();
             heap[ randN ].dist = (double) rand() / (RAND_MAX);
             if( !BH.updateHeap( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN << " not in heap" << std::endl;
             }
             BH.checkHeap();
         }


    }

    std::cout << "--------------------------------------------" << std::endl;

    // Tests heap operation function for returning the largest thing

    BH = BinaryHeap();
    srand(time(NULL));
    for( int i = 1; i <= 100; i++ ) {

         std::cout << i << std::endl;

         if( (r = (double) rand() / (RAND_MAX)) > 0.2 ) {
            std::cout << "add" << std::endl;
            KDTreeNode node = KDTreeNode(r);
            if( !BH.addToHeapB( &node ) ) {
                std::cout << "node not added to heap" << std::endl;
            }
         } else if( (r = (double) rand() / (RAND_MAX)) > 0.5 ) {
             std::cout << "pop" << std::endl;
             BH.popHeapB();
         } else if( *BH.getIndexOfLast() > 1.0 ) {
             std::cout << "remove" << std::endl;
             int randN = (int) rand() % (*BH.getIndexOfLast()) + 1;
             std::vector<KDTreeNode> heap = *BH.getHeap();
             BH.removeFromHeapB( &heap[ randN ] );
         }

         BH.checkHeapB();

         if( (r = (double) rand() / (RAND_MAX)) > 0.5 && *BH.getIndexOfLast() > 1.0 ) {
             std::cout << "update" << std::endl;
             int randN = (int) rand() % (*BH.getIndexOfLast()) + 1;
             std::vector<KDTreeNode>& heap = *BH.getHeap();
             heap[ randN ].dist = (double) rand() / (RAND_MAX);
             if( !BH.updateHeapB( heap[ randN ] ) ) {
                 std::cout << "node at index: " << randN << " not in heap" << std::endl;
             }
             BH.checkHeapB();
         }


    }
    return 0;
}*/
