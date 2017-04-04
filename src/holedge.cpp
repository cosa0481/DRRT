#include <DRRT/holedge.h>
#include <DRRT/kdtree.h>

/////////////////////// Static Edge Functions ///////////////////////
std::shared_ptr<Edge> Edge::NewEdge(std::shared_ptr<ConfigSpace> S,
                                    std::shared_ptr<KDTree> Tree,
                                    std::shared_ptr<KDTreeNode>& startNode,
                                    std::shared_ptr<KDTreeNode>& endNode)
{
    return std::make_shared<HolEdge>(S,Tree,startNode,endNode);
}

void Edge::Saturate(Eigen::VectorXd& nP,
                    Eigen::VectorXd cP,
                    double delta,
                    double dist)
{

}

/////////////////////// Virtual Edge Functions ///////////////////////
bool HolEdge::ValidMove()
{
    if( this->cspace_->space_has_time_ ) {
        // Note that planning happens in reverse time. i.e. time = 0 is at
        // the root of the search tree, and thus the time of startNode must be
        // greater than the time of endNode
        return ((this->start_node_->position_(2) > this->end_node_->position_(2))
                && ((this->cspace_->dubins_min_velocity_ <= this->velocity_)
                    && (this->velocity_ <= this->cspace_->dubins_max_velocity_)));
    }
    // if space does not have time then we assume that a move is always valid
    return true;
}

Eigen::VectorXd HolEdge::PoseAtDistAlongEdge(double distAlongEdge)
{
    double distRemaining = distAlongEdge;
    if( this->trajectory_.rows() < 2 || this->dist_ <= distAlongEdge ) {
        return this->end_node_->position_;
    }

    // Find the piece of trajectory_ that contains the point at the desired distance
    int i = 1;
    double thisDist = INF;
    // Check if 3rd column is not zero (3rd column is time)
    bool timeInPath = (this->trajectory_.col(2)(0) != 0.0);
    while( i <= this->trajectory_.rows() ) {
        double wtime = DubinsDistAlongTimePath( this->trajectory_.row(i-1),
                                                this->trajectory_.row(i) );
        double wotime = DubinsDistAlongPath( this->trajectory_.row(i-1),
                                             this->trajectory_.row(i) );
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
    Eigen::VectorXd ret = this->trajectory_.row(i-1)
            + ratio*(this->trajectory_.row(i)-this->trajectory_.row(i-1));
    double retTimeRatio = distAlongEdge/this->dist_;
    double retTime = this->start_node_->position_(2)
            + retTimeRatio*(this->end_node_->position_(2)
                            - this->start_node_->position_(2));
    double retTheta = atan2 (this->trajectory_(i,1) - this->trajectory_(i-1,1),
                             this->trajectory_(i,0) - this->trajectory_(i-1,0) );

    Eigen::VectorXd vec(4);
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

Eigen::VectorXd HolEdge::PoseAtTimeAlongEdge(double timeAlongEdge)
{
    if( this->trajectory_.rows() < 2 || (this->start_node_->position_(2)
                                        - this->end_node_->position_(2))
            <= timeAlongEdge ) {
        return this->end_node_->position_;
    }

    // Find the piece of the trajectory_ that contains the time at the
    // desired distance
    int i = 1;
    while( this->trajectory_(i,2)
           > this->start_node_->position_(2) - timeAlongEdge ) {
        i += 1;
    }

    // Now calculate pose along that piece
    double ratio = (this->trajectory_(i-1,2)
            - (this->start_node_->position_(2)-timeAlongEdge))
            / (this->trajectory_(i-1,2) - this->trajectory_(i,2));
    Eigen::VectorXd ret = this->trajectory_.row(i-1)
            + ratio*(this->trajectory_.row(i) - this->trajectory_.row(i-1));
    double retTime = this->start_node_->position_(2) - timeAlongEdge;
    double retTheta = atan2( this->trajectory_(i,1) - this->trajectory_(i-1,1),
                             this->trajectory_(i,0) - this->trajectory_(i-1,0) );

    Eigen::VectorXd vec(4);
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

void HolEdge::CalculateTrajectory()
{

}

void HolEdge::CalculateHoverTrajectory()
{

}

bool HolEdge::ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle)
{
    return false;
}
