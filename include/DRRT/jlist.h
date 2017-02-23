/* jlist.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef JLIST_H
#define JLIST_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <mutex>

class KDTreeNode;
class Edge;

// A JList node (not that key is unused for JList operations,
// but it is often helpful to have a key value associated
// with data)
class JListNode{
public:
    std::shared_ptr<JListNode> child;
    std::shared_ptr<JListNode> parent;
    std::shared_ptr<KDTreeNode> node;
    std::shared_ptr<Edge> edge;
    double key = 0.0;

    // Corstructor
    JListNode() : key(-1.0) {}
    JListNode(std::shared_ptr<KDTreeNode> t) : node(t) {}
    JListNode(std::shared_ptr<Edge> e) : edge(e) {}
};

// A simple JList
class JList{
public:
    std::shared_ptr<JListNode> front;
    std::shared_ptr<JListNode> back;
    std::shared_ptr<JListNode> bound;   // bounds either side of the list
    int length;
    bool useNodes;      // flag for indicating whether this JList
                        // is one of Edges or KDTreeNodes. True
                        // if JList uses KDTreeNodes

    // Constructor
    JList( bool nodeflag ) : useNodes(nodeflag)
    {
        std::shared_ptr<JListNode> endNode = std::make_shared<JListNode>();
        endNode->child = endNode;
        endNode->parent = endNode;

        front = endNode;
        back = endNode;
        bound = endNode;
        length = 0;
    }

    // Functions
    void JlistPush( std::shared_ptr<KDTreeNode> &t );
    void JlistPush( std::shared_ptr<Edge> &e );
    void JlistPush( std::shared_ptr<KDTreeNode> &t, double k );
    void JlistPush( std::shared_ptr<Edge> &e, double k );
    void JlistTop( std::shared_ptr<KDTreeNode> &t );
    void JlistTop( std::shared_ptr<Edge> &e );
    void JlistTopKey(std::shared_ptr<KDTreeNode> &n,
                     std::shared_ptr<double> k );
    void JlistTopKey( std::shared_ptr<Edge> &e, std::shared_ptr<double> k );
    void JlistPop( std::shared_ptr<KDTreeNode> &t );
    void JlistPop( std::shared_ptr<Edge> &e );
    void JlistPopKey(std::shared_ptr<KDTreeNode> &n,
                     std::shared_ptr<double> k);
    void JlistPopKey(std::shared_ptr<Edge> &e, std::shared_ptr<double> k);
    bool JlistRemove(std::shared_ptr<JListNode> &node );
    void JlistPrint();
    Eigen::Matrix<double, Eigen::Dynamic, 2> JlistAsMatrix();
    void JlistEmpty();
};

#endif // JLIST_H
