#include <DRRT/ghostPoint.h>
#include <DRRT/kdtree.h>

GhostPointIterator::GhostPointIterator(KDTree *t, Eigen::VectorXd qP)
    :   kd_tree_(t), query_point_(qP),
        ghost_tree_depth_(t->num_wraps_),
        current_ghost_(qP),
        closest_unwrapped_point_(qP)
   {
       Eigen::VectorXi zeros(t->num_wraps_);
       wrap_dim_flags_ = zeros;
   }


Eigen::VectorXd GetNextGhostPoint(std::shared_ptr<GhostPointIterator> G,
                                  double best_dist)
{
    // Will return out when done
    while( true ) {
        // Go up tree until we find a wrapDimFlags[ghostTreeDepth] == 0
        // (this indicates that we need to try permutations where ghost
        // is wrapped around KDTree.wraps_[ghostTreeDepth])
        while( G->ghost_tree_depth_ > 0
               && G->wrap_dim_flags_[G->ghost_tree_depth_-1] != 0 ) {
            G->ghost_tree_depth_ -= 1;
        }

        if( G->ghost_tree_depth_ == 0 ) {
            // We are finished, no more ghosts
            Eigen::VectorXd zeros;
            zeros.setZero();
            return zeros;
        }

        // Otherwise we are at depth where wrapDimFlags[ghostTreeDepth-1] == 0
        G->wrap_dim_flags_[G->ghost_tree_depth_-1] = 1;

        // Calculate this (wrapped) dimension of the ghost
        double wrapValue = G->kd_tree_->wrap_points_[G->ghost_tree_depth_-1];
        int wrapDim = G->kd_tree_->wraps_[G->ghost_tree_depth_-1];
        double dimVal = G->query_point_[wrapDim];
        double dimClosest = 0.0;
        if( G->query_point_[wrapDim] < wrapValue/2.0 ) {
            // Wrap to the right
            dimVal += wrapValue;
            dimClosest += wrapValue;
        } else {
            // Wrap to the left
            dimVal -= wrapValue;
        }
        G->current_ghost_[wrapDim] = dimVal;
        G->closest_unwrapped_point_[wrapDim] = dimClosest;

        // Finally move back down the tree to the lef-most possible leaf,
        // marking the path with 0s and populating appropriate dimension
        // of ghost point with values
        while( G->ghost_tree_depth_ < G->kd_tree_->num_wraps_ ) {
            G->ghost_tree_depth_ += 1;
            G->wrap_dim_flags_[G->ghost_tree_depth_-1] = 0;
            G->current_ghost_[G->kd_tree_->wraps_[G->ghost_tree_depth_-1]]
                = G->query_point_[G->kd_tree_->wraps_[G->ghost_tree_depth_-1]];
            G->closest_unwrapped_point_[
                    G->kd_tree_->wraps_[G->ghost_tree_depth_-1]]
              = G->current_ghost_[G->kd_tree_->wraps_[G->ghost_tree_depth_-1]];
        }

        // Check if closest point in unpwrapped space is further than
        // best distance
        if( G->kd_tree_->distanceFunction(G->closest_unwrapped_point_,
                                          G->current_ghost_) > best_dist ) {
            continue;
        }
        return G->current_ghost_;
    }
}
