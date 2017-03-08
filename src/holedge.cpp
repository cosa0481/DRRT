#include <DRRT/holedge.h>
#include <DRRT/kdtree.h>

/////////////////////// Static Edge Functions ///////////////////////
std::shared_ptr<Edge> Edge::newEdge(std::shared_ptr<CSpace> S,
                                    std::shared_ptr<KDTree> Tree,
                                    std::shared_ptr<KDTreeNode>& startNode,
                                    std::shared_ptr<KDTreeNode>& endNode)
{
    return std::make_shared<HolEdge>(S,Tree,startNode,endNode);
}

void Edge::saturate(Eigen::VectorXd& nP,
                    Eigen::VectorXd cP,
                    double delta,
                    double dist)
{

}

/////////////////////// Virtual Edge Functions ///////////////////////
bool HolEdge::ValidMove()
{
    if( this->cspace->spaceHasTime ) {
        // Note that planning happens in reverse time. i.e. time = 0 is at
        // the root of the search tree, and thus the time of startNode must be
        // greater than the time of endNode
        return ((this->startNode->position(2) > this->endNode->position(2))
                && ((this->cspace->dubinsMinVelocity <= this->velocity)
                    && (this->velocity <= this->cspace->dubinsMaxVelocity)));
    }
    // if space does not have time then we assume that a move is always valid
    return true;
}

Eigen::VectorXd HolEdge::poseAtDistAlongEdge(double distAlongEdge)
{
    double distRemaining = distAlongEdge;
    if( this->trajectory.rows() < 2 || this->dist <= distAlongEdge ) {
        return this->endNode->position;
    }

    // Find the piece of trajectory that contains the point at the desired distance
    int i = 1;
    double thisDist = INF;
    // Check if 3rd column is not zero (3rd column is time)
    bool timeInPath = (this->trajectory.col(2)(0) != 0.0);
    while( i <= this->trajectory.rows() ) {
        double wtime = dubinsDistAlongTimePath( this->trajectory.row(i-1),
                                                this->trajectory.row(i) );
        double wotime = dubinsDistAlongPath( this->trajectory.row(i-1),
                                             this->trajectory.row(i) );
        if( timeInPath ) {
            thisDist = wtime;
        } else {
            thisDist = wotime;
        }

        if( distRemaining - thisDist <= 0 ) {
            break;
        }

        distRemaining -= thisDist;
        i += 1;
    }

    if( distRemaining > thisDist ) {
        // In case of rare subtraction based precision errors
        distRemaining = thisDist;
    }

    // Now calculate pose along that piece
    double ratio = distRemaining/thisDist;
    Eigen::VectorXd ret = this->trajectory.row(i-1)
            + ratio*(this->trajectory.row(i)-this->trajectory.row(i-1));
    double retTimeRatio = distAlongEdge/this->dist;
    double retTime = this->startNode->position(2)
            + retTimeRatio*(this->endNode->position(2)
                            - this->startNode->position(2));
    double retTheta = atan2 (this->trajectory(i,1) - this->trajectory(i-1,1),
                             this->trajectory(i,0) - this->trajectory(i-1,0) );

    Eigen::VectorXd vec(4);
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

Eigen::VectorXd HolEdge::poseAtTimeAlongEdge(double timeAlongEdge)
{
    if( this->trajectory.rows() < 2 || (this->startNode->position(2)
                                        - this->endNode->position(2))
            <= timeAlongEdge ) {
        return this->endNode->position;
    }

    // Find the piece of the trajectory that contains the time at the
    // desired distance
    int i = 1;
    while( this->trajectory(i,2)
           > this->startNode->position(2) - timeAlongEdge ) {
        i += 1;
    }

    // Now calculate pose along that piece
    double ratio = (this->trajectory(i-1,2)
            - (this->startNode->position(2)-timeAlongEdge))
            / (this->trajectory(i-1,2) - this->trajectory(i,2));
    Eigen::VectorXd ret = this->trajectory.row(i-1)
            + ratio*(this->trajectory.row(i) - this->trajectory.row(i-1));
    double retTime = this->startNode->position(2) - timeAlongEdge;
    double retTheta = atan2( this->trajectory(i,1) - this->trajectory(i-1,1),
                             this->trajectory(i,0) - this->trajectory(i-1,0) );

    Eigen::VectorXd vec(4);
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

void HolEdge::calculateTrajectory()
{

}

void HolEdge::calculateHoverTrajectory()
{

}

bool HolEdge::ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle)
{
    return false;
}
