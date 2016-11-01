/* heap.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef HEAP_H
#define HEAP_H

#include <vector>
#include <iostream>
#include <DRRT/kdtreenode.h>



// A binary heap data structure
class BinaryHeap {
public:
    std::vector<KDTreeNode> H;  // stores the things that are in the heap
    int indexOfLast;            // the index of the last node in the heap array
    int parentOfLast;           // stores the index of the parent of the last node

    // Functions for interacting with marks and indices
    // Returns the key value of the node
    float key( KDTreeNode t ) { return t.dist; }
    // Default less than function
    bool lessThan( KDTreeNode a, KDTreeNode b ) { return (a.dist < b.dist); }
    // Default greater than function DATA
    bool greaterThan( KDTreeNode a, KDTreeNode b ) { return (a.dist > b.dist); }
    // Default heap marker function (marks when a node is in the heap)
    void mark( KDTreeNode* t ) { t->inHeap = true; }
    // Default heap unmarker function (un marks when a node is removed)
    void unmark( KDTreeNode* t ) { t->inHeap = false; }
    // Default heap check marker function (checks if node is marked)
    bool marked( KDTreeNode t ) { return t.inHeap; }
    // Sets the heap index to a value
    void setIndex( KDTreeNode* t, int value ) { t->heapIndex = value; }
    // Set the heap index to the unused value (-1)
    void unsetIndex( KDTreeNode* t ) { t->heapIndex = -1; }
    // Returns the heap index
    int getIndex( KDTreeNode t ) { return t.heapIndex; }

public:
    // Constructor
    BinaryHeap() : indexOfLast(0), parentOfLast(-1)
    {
        H.push_back( KDTreeNode() );
    }

    // Returns the heap in a vector array
    std::vector<KDTreeNode>* getHeap() { return &H; }
    // Returns the index of the last node in the heap array
    int* getIndexOfLast() { return &indexOfLast; }
    // Returns the index of the parent of the last index in the heap array
    int* getParentOfLast() { return &parentOfLast; }

    /* Heap operation functions for returning the smallest thing */

    // Compares a node with its parent and switches them if the parent's
    // cost is more than the node's cost. Repeats if a switch happens
    bool bubbleUp( int n )
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

    // Compares a node n with its childeren, and switches them if a child's
    // cost is less than the node's cost. Repeats if a switch happens
    bool bubbleDown( int n )
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

    // Add a node to the heap
    bool addToHeap( KDTreeNode* node )
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

    // Returns the node thas is on the top of the heap
    // If heap is empty, return node with data = -1
    KDTreeNode* topHeap()
    {
        if( indexOfLast < 1 ) return new KDTreeNode();
        return &H[1];
    }

    // Removes the top valued node from the heap and returns it
    // If heap is empty, return node with data = -1
    KDTreeNode popHeap()
    {
        if( indexOfLast < 1 ) return KDTreeNode();
        KDTreeNode oldTopNode = H[1];
        H[1] = H[indexOfLast];
        setIndex( &H[1], 1 );
        indexOfLast -= 1;
        parentOfLast = indexOfLast/2;
        bubbleDown( 1 );
        unmark( &oldTopNode );
        unsetIndex( &oldTopNode );
        return oldTopNode;
    }

    // Removes the node from the heap, assuming that it is in the heap
    bool removeFromHeap( KDTreeNode* node )
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

    // Updates a node that is already in the heap
    bool updateHeap( KDTreeNode node )
    {
        if( !marked( node ) ) {
            // Node not in the heap
            return false;
        }
        bubbleUp( getIndex( node ) );
        bubbleDown( getIndex( node ) );
        return true;
    }

    // Returns true if heap is good, false if bad
    bool checkHeap()
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

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    std::vector<KDTreeNode> cleanHeap()
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

    // Compares a node n with its parent, and switches them if
    // the parent's cost is less than the node's cost. Repeats
    // if a switch happens
    bool bubbleUpB( int n )
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

    // Compares an node n with its children, and switches them if
    // a child's cost is more than the node's cost. Repeats if
    // a switch happens
    bool bubbleDownB( int n )
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

    // Add node to the heap
    bool addToHeapB( KDTreeNode* node )
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

    // Returns the node that is on top of the heap
    KDTreeNode* topHeapB() { return topHeap(); }

    // Removes the top valued node from the heap and returns it
    KDTreeNode popHeapB()
    {
        if( indexOfLast < 1 ) return KDTreeNode();
        KDTreeNode oldTopNode = H[1];
        H[1] = H[indexOfLast];
        setIndex( &H[1], 1 );
        indexOfLast -= 1;
        parentOfLast = indexOfLast/2;
        bubbleDownB( 1 );
        unmark( &oldTopNode );
        unsetIndex( &oldTopNode );
        return oldTopNode;
    }

    // Removes the node from the heap, assuming it is in the heap
    bool removeFromHeapB( KDTreeNode* node )
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

    // Updates a node that is already in the heap
    bool updateHeapB( KDTreeNode node )
    {
        if( !marked( node ) ) {
            // Node not in the heap
            return false;
        }
        bubbleUpB( getIndex( node ) );
        bubbleDownB( getIndex( node ) );
        return true;
    }

    // Returns 1 if heap is good, 0 if bad
    bool checkHeapB()
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

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    std::vector<KDTreeNode> cleanHeapB() { return cleanHeap(); }
};

#endif // HEAP_H
