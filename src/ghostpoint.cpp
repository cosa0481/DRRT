#include <DRRT/ghostPoint.h>
#include <DRRT/kdtree.h>

ghostPointIterator::ghostPointIterator( KDTree* t, Eigen::VectorXd qP):
       kdTree(t), queryPoint(qP),
       ghostTreeDepth(t->numWraps),
       currentGhost(qP),
       closestUnwrappedPoint(qP)
   {
       Eigen::VectorXi zeros(t->numWraps,0);
       wrapDimFlags = zeros;
   }


Eigen::VectorXd getNextGhostPoint( ghostPointIterator G, double bestDist )
{
    // Will return out when done
    while( true ) {
        // Go up tree until we find a wrapDimFlags[ghostTreeDepth] == 0
        // (this indicates that we need to try permutations where ghost
        // is wrapped around KDTree.wraps[ghostTreeDepth])

        while( G.ghostTreeDepth > 0 && G.wrapDimFlags[G.ghostTreeDepth] != 0 ) {
            G.ghostTreeDepth -= 1;
        }

        if( G.ghostTreeDepth == 0 ) {
            // We are finished, no more ghosts
            Eigen::VectorXd zeros(G.kdTree->numWraps,0);
            return zeros;
        }

        // Otherwise we are at depth where wrapDimFlags[ghostTreeDepth] == 0
        G.wrapDimFlags[G.ghostTreeDepth] = 1;

        // Calculate this (wrapped) dimension of the ghost
        double dimVal = G.queryPoint[G.kdTree->wrapPoints[G.ghostTreeDepth]];
        double dimClosest = 0.0;
        if( G.queryPoint[G.kdTree->wraps[G.ghostTreeDepth]] < G.kdTree->wrapPoints[G.ghostTreeDepth]/2.0 ) {
            // Wrap to the right
            dimVal += G.kdTree->wrapPoints[G.ghostTreeDepth];
            dimClosest += G.kdTree->wrapPoints[G.ghostTreeDepth];
        } else {
            // Wrap to the left
            dimVal -= G.kdTree->wrapPoints[G.ghostTreeDepth];
            G.currentGhost[G.kdTree->wraps[G.ghostTreeDepth]] = dimVal;
            G.closestUnwrappedPoint[G.kdTree->wraps[G.ghostTreeDepth]] = dimClosest;
        }

        // Finally move back down the tree to the lef-most possible leaf,
        // marking the path with 0s and populating appropriate dimension
        // of ghost point with values
        while( G.ghostTreeDepth < G.kdTree->numWraps ) {
            G.ghostTreeDepth += 1;
            G.wrapDimFlags[G.ghostTreeDepth] = 0;
            G.currentGhost[G.kdTree->wraps[G.ghostTreeDepth]] = G.queryPoint[G.kdTree->wraps[G.ghostTreeDepth]];
            G.closestUnwrappedPoint[G.kdTree->wraps[G.ghostTreeDepth]] = G.currentGhost[G.kdTree->wraps[G.ghostTreeDepth]];
        }

        // Check if closest point in unpwrapped space is further than
        // best distance
        if( distFunc(G.kdTree->distanceFunction,
                     G.closestUnwrappedPoint,
                     G.currentGhost
                     ) > bestDist ) {
            continue;
        }
        return G.currentGhost;
    }
}
