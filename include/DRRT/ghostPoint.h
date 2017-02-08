/* ghostPoint.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef GHOSTPOINT_H
#define GHOSTPOINT_H

#include <eigen3/Eigen/Eigen>

class KDTree;

/* This contains an iterator that efficiently gives
 * the next ghost point needed for searching in a kd
 * tree that has some dimensions that wrap around.
 * Note that it starts at the first ghost and does
 * not return the original point.
 */
typedef struct ghostPointIterator {
    std::shared_ptr<KDTree> kdTree;                // the kd tree that this is being used with
    Eigen::VectorXd queryPoint;    // the actual point that all the ghost are "identical" to DATA
    Eigen::VectorXi wrapDimFlags;  // the current permutation
    int ghostTreeDepth;   // pointer to the current depth of the "ghost tree"
                 //      Note that this "tree" is only a theoretical construct
                 //      that determines the order in which the ghosts are
                 //      returned. It should not be confused with the kdtree.
    Eigen::VectorXd currentGhost;    // the current ghost that we are returning
    Eigen::VectorXd closestUnwrappedPoint;  // the closest point in the normal
                 // space to the currentGhost. Dist between this and ghost can
                 // can be used as heuristic to skip unhelpful ghosts

    // Constructor
    ghostPointIterator( std::shared_ptr<KDTree> t, Eigen::VectorXd qP);
} ghostPointIterator;

// This returns the next ghost point, not that it starts
// at the first -ghost- and does not return the original point DATA
Eigen::VectorXd getNextGhostPoint(std::shared_ptr<ghostPointIterator> G,
                                  double bestDist);

#endif // GHOSTPOINT_H
