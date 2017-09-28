#ifndef GHOST_POINT_H
#define GHOST_POINT_H

#include <DRRT/libraries.h>

class KdTree;

typedef struct GhostPointIterator {
    std::shared_ptr<KdTree> tree;
    Eigen::VectorXd query;

    Eigen::VectorXi wrap_dim_flags;
    int ghost_tree_depth;

    Eigen::VectorXd current_ghost;
    Eigen::VectorXd closest_unwrapped_point;

    GhostPointIterator(std::shared_ptr<KdTree> kdtree,
                       Eigen::VectorXd query_point);

    Eigen::VectorXd GetNextGhostPoint(double best_dist);
} GhostPointIterator;

typedef std::shared_ptr<GhostPointIterator> GhostPointIterator_ptr;

#endif // GHOST_POINT_H
