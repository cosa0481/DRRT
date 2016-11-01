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
}
