#include <DRRT/ghost_point.h>
#include <DRRT/kdtree.h>

GhostPointIterator::GhostPointIterator(std::shared_ptr<KdTree> kdtree,
                                       Eigen::VectorXd query_point)
    : tree(kdtree), query(query_point),
      wrap_dim_flags(Eigen::VectorXi(kdtree->num_wraps_).setZero()),
      ghost_tree_depth(kdtree->num_wraps_), current_ghost(query),
      closest_unwrapped_point(query)
{}

Eigen::VectorXd GhostPointIterator::GetNextGhostPoint(double best_dist)
{
    while(true)
    {
        // Go up tree until we need to try permutations where ghost
        // is wrapped around tree->wraps_[ghost_tree_depth]
        while(ghost_tree_depth > 0 && wrap_dim_flags[ghost_tree_depth - 1] != 0) {
            ghost_tree_depth -= 1;
        }

        // No more ghosts
        if(ghost_tree_depth == 0)
            return Eigen::VectorXd().setZero();

        wrap_dim_flags[ghost_tree_depth - 1] = 1;

        // Calculate the wrapped dimension of the ghost
        double wrap_value = tree->wrap_points_[ghost_tree_depth - 1];
        int wrap_dim = tree->wraps_[ghost_tree_depth - 1];
        double dim_val = query[wrap_dim];
        double dim_closest = 0.0;
        if(query[wrap_dim] < wrap_value/2.0) {
            // Wrap to right
            dim_val += wrap_value;
            dim_closest += wrap_value;
        } else {
            // Wrap to left
            dim_val -= wrap_value;
        }

        current_ghost[wrap_dim] = dim_val;
        closest_unwrapped_point[wrap_dim] = dim_closest;

        // Move back down tree to the left-most leaf, marking the path
        // with 0s and populating the appropriate dimension of ghost point
        while(ghost_tree_depth < tree->num_wraps_) {
            ghost_tree_depth += 1;
            wrap_dim_flags[ghost_tree_depth - 1] = 0;
            current_ghost[tree->wraps_[ghost_tree_depth - 1]]
                    = query[tree->wraps_[ghost_tree_depth - 1]];
            closest_unwrapped_point[tree->wraps_[ghost_tree_depth - 1]]
                    = current_ghost[tree->wraps_[ghost_tree_depth - 1]];
        }

        // Check if closest point in unwrapped space is further than best distance
        if(DistanceFunction(closest_unwrapped_point, current_ghost) > best_dist)
            continue;

        return current_ghost;
    }
}
