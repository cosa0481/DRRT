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
typedef struct GhostPointIterator {
    KDTree* kd_tree_; // the kd tree that this is being used with
    Eigen::VectorXd query_point_; // the actual point to which
                                  // all the ghosts are "identical"
    Eigen::VectorXi wrap_dim_flags_; // the current permutation
    int ghost_tree_depth_; // current depth of the "ghost tree"
                 //      Note that this "tree" is only a theoretical construct
                 //      that determines the order in which the ghosts are
                 //      returned. It should not be confused with the kdtree.
    Eigen::VectorXd current_ghost_; // the current ghost that we are returning
    Eigen::VectorXd closest_unwrapped_point_; // the closest point in the
                 // normal space to the currentGhost. Dist between this and
                 // ghost can can be used as heuristic to skip unhelpful ghosts

    // Constructor
    GhostPointIterator(KDTree *t, Eigen::VectorXd qP);
} GhostPointIterator;

// This returns the next ghost point, not that it starts
// at the first -ghost- and does not return the original point
Eigen::VectorXd GetNextGhostPoint(std::shared_ptr<GhostPointIterator> G,
                                  double best_dist);

#endif // GHOSTPOINT_H
