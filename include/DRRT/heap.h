/* heap.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef HEAP_H
#define HEAP_H

#include <DRRT/kdtreenode.h>

// A binary heap data structure
class BinaryHeap {
public:
    // Stores the things that are in the heap
    std::vector<std::shared_ptr<KDTreeNode>> heap_;
    int index_of_last_;    // the index of the last node in the heap array
    int parent_of_last_;   // stores the index of the parent of the last node
    bool use_default_;     // flag for using key or keyQ e.g. (DRRT.jl>>3289)

    // Functions for interacting with marks and indices
    // Returns the key value of the node
    /// Should keyD return rrt_LMC_ as well? used in KDFindKNearest only (afaik)
    double keyD(std::shared_ptr<KDTreeNode> node)
    {return node->dist_;}

    double keyQ(std::shared_ptr<KDTreeNode> node)
    {return std::min( node->rrt_tree_cost_, node->rrt_LMC_ );}

    double key(std::shared_ptr<KDTreeNode> node)
    {return ( use_default_ ? keyD(node) : keyQ(node) );}


    // Default less than function
    bool lessD(std::shared_ptr<KDTreeNode> a, std::shared_ptr<KDTreeNode> b)
    {return (keyD(a) < keyD(b));}

    bool lessQ(std::shared_ptr<KDTreeNode> a, std::shared_ptr<KDTreeNode> b)
    {return ((keyQ(a) < keyQ(b))
             || (keyQ(a) == keyQ(b) && a->is_move_goal_));}

    bool lessThan(std::shared_ptr<KDTreeNode> a,
                  std::shared_ptr<KDTreeNode> b)
    {return ( use_default_ ? lessD(a,b) : lessQ(a,b) );}


    // Default greater than function DATA
    bool greaterD(std::shared_ptr<KDTreeNode> a, std::shared_ptr<KDTreeNode> b)
    {return (keyD(a) > keyD(b));}

    bool greaterQ(std::shared_ptr<KDTreeNode> a, std::shared_ptr<KDTreeNode> b)
    {return ((keyQ(a) > keyQ(b))
             || (keyQ(a) == keyQ(b) && b->is_move_goal_));}

    bool greaterThan(std::shared_ptr<KDTreeNode> a,
                     std::shared_ptr<KDTreeNode> b)
    {return ( use_default_ ? greaterD(a,b) : greaterQ(a,b) );}


    // Default heap marker function (marks when a node is in the heap)
    void markD(std::shared_ptr<KDTreeNode> &node)
    {node->in_heap_ = true;}

    void markQ(std::shared_ptr<KDTreeNode> &node)
    {node->in_priority_queue_ = true;}

    void mark(std::shared_ptr<KDTreeNode> &node)
    {(use_default_ ? markD(node) : markQ(node));}


    // Default heap unmarker function (un marks when a node is removed)
    void unmarkD(std::shared_ptr<KDTreeNode> &node)
    {node->in_heap_ = false;}

    void unmarkQ(std::shared_ptr<KDTreeNode> &node)
    {node->in_priority_queue_ = false;}

    void unmark(std::shared_ptr<KDTreeNode> &node)
    {(use_default_ ? unmarkD(node) : unmarkQ(node));}


    // Default heap check marker function (checks if node is marked)
    bool markedD(std::shared_ptr<KDTreeNode> node)
    {return node->in_heap_;}

    bool markedQ(std::shared_ptr<KDTreeNode> node)
    {return node->in_priority_queue_;}

    bool marked(std::shared_ptr<KDTreeNode> node)
    {return (use_default_ ? markedD(node) : markedQ(node));}


    // Sets the heap index to a value
    void setIndexD(std::shared_ptr<KDTreeNode> &node, int value)
    {node->heap_index_ = value;}

    void setIndexQ(std::shared_ptr<KDTreeNode> &node, int value)
    {node->priority_queue_index_ = value;}

    void setIndex(std::shared_ptr<KDTreeNode> &node, int value)
    {(use_default_ ? setIndexD(node,value) : setIndexQ(node,value));}


    // Set the heap index to the unused value (-1)
    void unsetIndexD(std::shared_ptr<KDTreeNode> &node)
    {node->heap_index_ = -1;}

    void unsetIndexQ(std::shared_ptr<KDTreeNode> &node)
    {node->priority_queue_index_ = -1;}

    void unsetIndex(std::shared_ptr<KDTreeNode> &node)
    {(use_default_ ? unsetIndexD(node) : unsetIndexQ(node));}


    // Returns the heap index
    int getIndexD(std::shared_ptr<KDTreeNode> node)
    {return node->heap_index_;}

    int getIndexQ(std::shared_ptr<KDTreeNode> node)
    {return node->priority_queue_index_;}

    int getIndex(std::shared_ptr<KDTreeNode> node)
    {return (use_default_ ? getIndexD(node) : getIndexQ(node));}


    // Constructor
    BinaryHeap(bool use_d) : index_of_last_(0), parent_of_last_(-1), use_default_(use_d)
    {
        this->heap_.push_back( std::make_shared<KDTreeNode>() );
    }

    // Displays the KDTreeNodes in the heap
    void DisplayHeap();

    // Returns the heap in a vector array
    void GetHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap);
    // Returns the index of the last node in the heap array
    int* GetIndexOfLast() {return &index_of_last_;}
    // Returns the index of the parent of the last index in the heap array
    int* GetParentOfLast() {return &parent_of_last_;}

    /* Heap operation functions for returning the smallest thing */

    // Compares a node with its parent and switches them if the parent's
    // cost is more than the node's cost. Repeats if a switch happens
    bool BubbleUp(int n);

    // Compares a node n with its childeren, and switches them if a child's
    // cost is less than the node's cost. Repeats if a switch happens
    bool BubbleDown(int n);

    // Add a node to the heap
    bool AddToHeap(std::shared_ptr<KDTreeNode> &node);

    // Returns the node thas is on the top of the heap
    // If heap is empty, return node with data = -1
    void TopHeap(std::shared_ptr<KDTreeNode> &node);

    // Removes the top valued node from the heap and returns it
    // If heap is empty, return node with data = -1
    void PopHeap(std::shared_ptr<KDTreeNode> &node);

    // Removes the node from the heap, assuming that it is in the heap
    bool RemoveFromHeap(std::shared_ptr<KDTreeNode> &node);

    // Updates a node that is already in the heap
    bool UpdateHeap(std::shared_ptr<KDTreeNode> &node);

    // Returns true if heap is good, false if bad
    bool CheckHeap();

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    void CleanHeap(std::vector<std::shared_ptr<KDTreeNode>> &heap);

    /* Heap operation functions for returning the largest thing */

    // Compares a node n with its parent, and switches them if
    // the parent's cost is less than the node's cost. Repeats
    // if a switch happens
    bool BubbleUpB(int n);

    // Compares an node n with its children, and switches them if
    // a child's cost is more than the node's cost. Repeats if
    // a switch happens
    bool BubbleDownB(int n);

    // Add node to the heap
    bool AddToHeapB(std::shared_ptr<KDTreeNode> &node);

    // Returns the node that is on top of the heap
    void TopHeapB(std::shared_ptr<KDTreeNode> &node) {return TopHeap(node);}

    // Removes the top valued node from the heap and returns it
    void PopHeapB(std::shared_ptr<KDTreeNode> &node);

    // Removes the node from the heap, assuming it is in the heap
    bool RemoveFromHeapB(std::shared_ptr<KDTreeNode> node);

    // Updates a node that is already in the heap
    bool UpdateHeapB(std::shared_ptr<KDTreeNode> node);

    // Returns 1 if heap is good, 0 if bad
    bool CheckHeapB();

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    void CleanHeapB(std::vector<std::shared_ptr<KDTreeNode>> &heap)
    { return CleanHeap(heap); }
};

#endif // HEAP_H
