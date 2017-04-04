#include <DRRT/dubinsedge.h>
#include <DRRT/drrt.h>

using namespace std;

/////////////////////// Critical Functions ///////////////////////

/////////////////////// Static Edge Functions ///////////////////////
std::shared_ptr<Edge> Edge::NewEdge(std::shared_ptr<ConfigSpace> C,
                                    std::shared_ptr<KDTree> Tree,
                                    std::shared_ptr<KDTreeNode>& start_node,
                                    std::shared_ptr<KDTreeNode>& end_node)
{
    return std::make_shared<DubinsEdge>(C,Tree,start_node,end_node);
}

// For the Dubin's Car Model, Saturate x,y,theta
void Edge::Saturate(Eigen::VectorXd& nP,
                    Eigen::VectorXd cP,
                    double delta,
                    double distance)
{
    // First scale non-theta dimensions
    nP.head(2) = cP.head(2) +
            ( nP.head(2) - cP.head(2) ) * delta / distance;

    while(nP(2) < -2*PI) nP(2) += 2*PI;
    while(nP(2) > 2*PI) nP(2) -= 2*PI;

    ///std::cout << "modded theta: " << nP(2) << std::endl;

//    // Saturate theta in the shorter of the
//    // two directions that it can go
//    if( std::abs( nP(2) - cP(2) ) < PI ) {
//        // Saturate in the normal way
//        nP(2) = cP(2) +
//                (nP(2) - cP(2)) * delta / dist;
//    } else {
//        // Saturate in the opposite way
//        if( nP(2) < PI ) {
//            nP(2) = nP(2) + 2*PI;
//        } else {
//            nP(2) = nP(2) - 2*PI;
//        }

//        // Now Saturate
//        nP(2) = cP(2) +
//                (nP(2) - cP(2)) * delta / dist;

//        // Finally, wrap back to the identity that is on [0 2pi]
//        // Is this really wrapping?

//        Eigen::Vector2d minVec;
//        minVec(0) = nP(2);
//        minVec(1) = 2*PI;
//        Eigen::Vector2d maxVec;
//        maxVec(0) = minVec.minCoeff();
//        maxVec(1) = 0.0;

//        nP(2) = maxVec.maxCoeff();
//        std::cout << "Saturated theta: " << nP(2) << std::endl;
//    }
}

/////////////////////// Virtual Edge Functions ///////////////////////
bool DubinsEdge::ValidMove()
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

Eigen::VectorXd DubinsEdge::PoseAtDistAlongEdge(double dist_along_edge)
{
    double distRemaining = dist_along_edge;
    if( this->trajectory_.rows() < 2 || this->dist_ <= dist_along_edge ) {
        return this->end_node_->position_;
    }

    // Find the piece of trajectory that contains the point
    // at the desired distance
    int i = 1;
    double thisDist = INF;
    bool timeInPath = this->trajectory_.cols() > 3;
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
    double retTheta = atan2 (this->trajectory_(i,1) - this->trajectory_(i-1,1),
                             this->trajectory_(i,0) - this->trajectory_(i-1,0) );

    Eigen::Vector3d vec;
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTheta;

    return vec;
}

Eigen::VectorXd DubinsEdge::PoseAtTimeAlongEdge(double time_along_edge)
{
    if( this->trajectory_.rows() < 2 || (this->start_node_->position_(2)
                                        - this->end_node_->position_(2))
            <= time_along_edge ) {
        return this->end_node_->position_;
    }

    // Find the piece of the trajectory_ that contains the time at the
    // desired distance
    int i = 1;
    while( this->trajectory_(i,2)
           > this->start_node_->position_(2) - time_along_edge ) {
        i += 1;
    }

    // Now calculate pose along that piece
    double ratio = (this->trajectory_(i-1,2)
                    - (this->start_node_->position_(2)-time_along_edge))
            / (this->trajectory_(i-1,2) - this->trajectory_(i,2));
    Eigen::VectorXd ret = this->trajectory_.row(i-1)
            + ratio*(this->trajectory_.row(i) - this->trajectory_.row(i-1));
    double retTime = this->start_node_->position_(2) - time_along_edge;
    double retTheta = atan2( this->trajectory_(i,1) - this->trajectory_(i-1,1),
                             this->trajectory_(i,0) - this->trajectory_(i-1,0) );

    Eigen::VectorXd vec;
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

void DubinsEdge::CalculateTrajectory()
{
    double r_min = this->cspace_->min_turn_radius_;

    Eigen::Vector2d initial_location = this->start_node_->position_.head(2);
    double initial_theta = this->start_node_->position_(2);
    Eigen::Vector2d goal_location = this->end_node_->position_.head(2);
    double goal_theta = this->end_node_->position_(2);

    Eigen::Vector2d temp, temp2;

    // Calculate the center of the right-turn and left-turn circles
    // Right-turn initial_location circle
    temp(0) = cos(initial_theta-(PI/2.0));
    temp(1) = sin(initial_theta-(PI/2.0));
    Eigen::Vector2d irc_center = initial_location + r_min * temp;

    // Left-turn initial location circle
    temp(0) = cos(initial_theta+(PI/2.0));
    temp(1) = sin(initial_theta+(PI/2.0));
    Eigen::Vector2d ilc_center = initial_location + r_min * temp;

    // Right-turn goal_location circle
    temp(0) = cos(goal_theta-(PI/2.0));
    temp(1) = sin(goal_theta-(PI/2.0));
    Eigen::Vector2d grc_center = goal_location + r_min * temp;

    // Left-turn goal_location circle
    temp(0) = cos(goal_theta+(PI/2.0));
    temp(1) = sin(goal_theta+(PI/2.0));
    Eigen::Vector2d glc_center = goal_location + r_min * temp;


    // Remember the best distance and associated type of trajectory
    double bestDist = INF;
    std::string bestTrajType = "xxx";

    Eigen::ArrayXd diff, temp3(2), t; // Arrays used for coefficient-wise
                                      // operations
    double D, R, sq, a, b, firstDist, secondDist, thirdDist;
    Eigen::VectorXd v;

    /// BEGIN determine best path

    // Calculate tangent points for right-straight-left path
    // r-s-l requires "inner" tangent points
    diff = glc_center - irc_center;
    D = sqrt((diff*diff).sum());
    v = diff/D;
    R = -2.0*r_min/D;
    double rsl_length;
    Eigen::Vector2d rsl_tangent_x, rsl_tangent_y;
    if( std::abs(R) > 1.0 ) {
        rsl_length = INF;
    } else {
        sq = sqrt(1.0-R*R);
        a = r_min*(R*v(0) + v(1)*sq);
        b = r_min*(R*v(1) - v(0)*sq);
        rsl_tangent_x(0) = irc_center(0) - a;
        rsl_tangent_x(1) = glc_center(0) + a;
        rsl_tangent_y(0) = irc_center(1) - b;
        rsl_tangent_y(1) = glc_center(1) + b;

        temp(0) = rsl_tangent_x(0);
        temp(1) = rsl_tangent_y(0);
        firstDist = RightTurnDist( initial_location, temp, irc_center, r_min );
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        temp2(0) = rsl_tangent_x(0);
        temp2(1) = rsl_tangent_y(0);
        diff = temp - temp2;
        secondDist = sqrt((diff*diff).sum());
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        thirdDist = LeftTurnDist( temp, goal_location, glc_center, r_min );

        rsl_length = firstDist + secondDist + thirdDist;

        if( bestDist > rsl_length ) {
            bestDist = rsl_length;
            bestTrajType = "rsl";
        }
    }


    // Calculate tangent points for right-straight-right path
    // r-s-r requires "outer" tangent points, and the tangent vector
    // is parallel to the vector from irc to grc
    diff = grc_center - irc_center;
    D = sqrt((diff*diff).sum());
    v = diff/D;
    double rsr_length;
    Eigen::Vector2d  rsr_tangent_x, rsr_tangent_y;
    temp3(0) = irc_center(0);
    temp3(1) = grc_center(0);
    rsr_tangent_x = -r_min*v(1) + temp3;
    temp3(0) = irc_center(1);
    temp3(1) = grc_center(1);
    rsr_tangent_y = r_min*v(0) + temp3;

    temp(0) = rsr_tangent_x(0);
    temp(1) = rsr_tangent_y(0);
    firstDist = RightTurnDist( initial_location, temp, irc_center, r_min );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    temp2(0) = rsr_tangent_x(0);
    temp2(1) = rsr_tangent_y(0);
    diff = temp - temp2;
    secondDist = sqrt( (diff*diff).sum() );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    thirdDist = RightTurnDist( temp, goal_location, grc_center, r_min );

    rsr_length = firstDist + secondDist + thirdDist;

    if( bestDist > rsr_length ) {
        bestDist = rsr_length;
        bestTrajType = "rsr";
    }


    // Calculate (if advantagous) the right-left-right path
    double rlr_length;
    Eigen::Array2d rlr_rl_tangent, rlr_lr_tangent;
    rlr_rl_tangent(0) = 0;
    rlr_rl_tangent(1) = 0;
    rlr_lr_tangent(0) = 0;
    rlr_lr_tangent(1) = 0;

    // D was last set by the rsr calculation
    Eigen::Vector2d rlr_l_circle_center;
    if( D < 4.0*r_min ) {
        // Start by finding the center of the "left" circle
        double theta = -acos(D/(4*r_min)) + atan2(v(1),v(0));
        temp3(0) = cos(theta);
        temp3(1) = sin(theta);
        t = irc_center;
        rlr_l_circle_center = t + 2*r_min*temp3;
        rlr_rl_tangent = (rlr_l_circle_center + irc_center)/2.0;
        rlr_lr_tangent = (rlr_l_circle_center + grc_center)/2.0;

        firstDist = RightTurnDist( initial_location, rlr_rl_tangent,
                                   irc_center, r_min );
        secondDist = LeftTurnDist( rlr_rl_tangent, rlr_lr_tangent,
                                   rlr_l_circle_center, r_min );
        thirdDist = RightTurnDist( rlr_lr_tangent, goal_location,
                                   grc_center, r_min );

        rlr_length = firstDist + secondDist + thirdDist;

        if( bestDist > rlr_length ) {
            bestDist = rlr_length;
            bestTrajType = "rlr";
        } else {
            rlr_length = INF;
        }
    }


    // Calculate tangent points for left-straight-right path
    // l-s-r requires "inner" tangent points
    diff = grc_center - ilc_center;
    D = sqrt( (diff*diff).sum() );
    v = diff/D;
    R = 2.0*r_min/D;
    double lsr_length;
    Eigen::Vector2d lsr_tangent_x, lsr_tangent_y;
    if( std::abs(R) > 1.0 ) {
        lsr_length = INF;
    } else {
        sq = sqrt(1.0-R*R);
        a = R*v(0) + v(1)*sq;
        b = R*v(1) - v(0)*sq;
        lsr_tangent_x(0) = ilc_center(0) + a*r_min;
        lsr_tangent_x(1) = grc_center(0) - a*r_min;
        lsr_tangent_y(0) = ilc_center(1) + b*r_min;
        lsr_tangent_y(1) = grc_center(1) - b*r_min;

        temp(0) = lsr_tangent_x(0);
        temp(1) = lsr_tangent_y(0);
        firstDist = LeftTurnDist( initial_location, temp, ilc_center, r_min );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        temp2(0) = lsr_tangent_x(0);
        temp2(1) = lsr_tangent_y(0);
        diff = temp - temp2;
        secondDist = sqrt( (diff*diff).sum() );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        thirdDist = RightTurnDist( temp, goal_location, grc_center, r_min );

        lsr_length = firstDist + secondDist + thirdDist;

        if( bestDist > lsr_length ) {
            bestDist = lsr_length;
            bestTrajType = "lsr";
        }
    }


    // Calculate tangent points for left-straight-left path
    // l-s-l requires "outer" tangent poinst, and the tangent
    // vector is parallel to the vector from irc to grc
    double lsl_length;
    Eigen::Vector2d lsl_tangent_x, lsl_tangent_y;
    diff = glc_center - ilc_center;
    D = sqrt( (diff*diff).sum() );
    v = diff/D;
    temp3(0) = ilc_center(0);
    temp3(1) = glc_center(0);
    lsl_tangent_x = r_min*v(1) + temp3;
    temp3(0) = ilc_center(1);
    temp3(1) = glc_center(1);
    lsl_tangent_y = -r_min*v(0) + temp3;

    temp(0) = lsl_tangent_x(0);
    temp(1) = lsl_tangent_y(0);
    firstDist = LeftTurnDist( initial_location, temp, ilc_center, r_min );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    temp2(0) = lsl_tangent_x(0);
    temp2(1) = lsl_tangent_y(0);
    diff = temp - temp2;
    secondDist = sqrt( (diff*diff).sum() );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    thirdDist = LeftTurnDist( temp, goal_location, glc_center, r_min );

    lsl_length = firstDist + secondDist + thirdDist;

    if( bestDist > lsl_length ) {
        bestDist = lsl_length;
        bestTrajType = "lsl";
    }


    // Calculate (if advantagous) the left-right-left path
    double lrl_length;
    Eigen::Array2d lrl_rl_tangent, lrl_lr_tangent;
    lrl_rl_tangent(0) = 0;
    lrl_rl_tangent(1) = 0;
    lrl_lr_tangent(0) = 0;
    lrl_lr_tangent(1) = 0;

    // D was last set by the lsl calculation
    Eigen::Vector2d lrl_r_circle_center;
    if( D < 4.0*r_min ) {
        // Start by finding the center of the "left" circle
        double theta = -acos(D/(4*r_min)) + atan2(v(1),v(0));
        temp3(0) = cos(theta);
        temp3(1) = sin(theta);
        t = ilc_center;
        lrl_r_circle_center = t + 2*r_min*temp3;
        lrl_lr_tangent = (lrl_r_circle_center + ilc_center)/2.0;
        lrl_rl_tangent = (lrl_r_circle_center + glc_center)/2.0;

        firstDist = RightTurnDist( initial_location, lrl_lr_tangent,
                                   ilc_center, r_min );
        secondDist = LeftTurnDist( lrl_lr_tangent, lrl_rl_tangent,
                                   lrl_r_circle_center, r_min );
        thirdDist = RightTurnDist( lrl_rl_tangent, goal_location,
                                   glc_center, r_min );

        lrl_length = firstDist + secondDist + thirdDist;

        if( bestDist > lrl_length ) {
            bestDist = lrl_length;
            bestTrajType = "lrl";
        } else {
            lrl_length = INF;
        }
    }

    /// END determine best path
    /// BEGIN calculate trajectory from path
    // Now save the best path in the trajectory field

    double delta_phi = 0.1; // this is the angle granularity (in radians)
                            // used for discritizing the arcs of the paths
                            // (straight lines are saved as a single segment

    Eigen::ArrayXd phis;    // This array contains 1 element or a sequence of elements
    Eigen::Vector2d p, p1, p2;
    Eigen::VectorXd first_path_x, first_path_y,
                    second_path_x, second_path_y,
                    third_path_x, third_path_y;
    double phi_start, phi_end, val;
    int i;

    // Calculate the first part of the path
    if( bestTrajType[0] == 'r' ) {
        // First part of the path is a right hand turn
        if( bestTrajType == "rsl" ) {
            p(0) = rsl_tangent_x(0);
            p(1) = rsl_tangent_y(0);
        } else if( bestTrajType == "rsr" ) {
            p(0) = rsr_tangent_x(0);
            p(1) = rsr_tangent_y(0);
        } else { // bestTrajType == "rlr"
            p = rlr_rl_tangent;
        }

        phi_start = atan2( initial_location(1)-irc_center(1),
                           initial_location(0)-irc_center(0) );
        phi_end = atan2( p(1)-irc_center(1), p(0)-irc_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val >= phi_end ) {
                phis(i) = val;
                val -= delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        first_path_x = irc_center(0) + r_min*phis.cos();
        first_path_y = irc_center(1) + r_min*phis.sin();
    } else if( bestTrajType[0] == 'l' ) {
        // First part of the path is a left hand turn
        if( bestTrajType == "lsl" ) {
            p(0) = lsl_tangent_x(0);
            p(1) = lsl_tangent_y(0);
        } else if( bestTrajType == "lsr" ) {
            p(0) = lsr_tangent_x(0);
            p(1) = lsr_tangent_y(0);
        } else { // bestTrajType == "lrl"
            p = lrl_lr_tangent;
        }

        phi_start = atan2( initial_location(1)-ilc_center(1),
                           initial_location(0)-ilc_center(0) );
        phi_end = atan2( p(1)-ilc_center(1), p(0)-ilc_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val <= phi_end ) {
                phis(i) = val;
                val += delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        first_path_x = ilc_center(0) + r_min*phis.cos();
        first_path_y = ilc_center(1) + r_min*phis.sin();
    }


    // Calculate the second part of the path
    if( bestTrajType[1] == 's' ) {
        // Second part of the path is a straight line
        if( bestTrajType == "lsr" ) {
            p1(0) = lsr_tangent_x(0);
            p1(1) = lsr_tangent_y(0);
            p2(0) = lsr_tangent_x(1);
            p2(1) = lsr_tangent_y(1);
        } else if( bestTrajType == "lsl" ) {
            p1(0) = lsl_tangent_x(0);
            p1(1) = lsl_tangent_y(0);
            p2(0) = lsl_tangent_x(1);
            p2(1) = lsl_tangent_y(1);
        } else if( bestTrajType == "rsr" ) {
            p1(0) = rsr_tangent_x(0);
            p1(1) = rsr_tangent_y(0);
            p2(0) = rsr_tangent_x(1);
            p2(1) = rsr_tangent_y(1);
        } else { // bestTrajType == "rsl"
            p1(0) = rsl_tangent_x(0);
            p1(1) = rsl_tangent_y(0);
            p2(0) = rsl_tangent_x(1);
            p2(1) = rsl_tangent_y(1);
        }

        Eigen::Vector2d ptemp1, ptemp2;

        ptemp1 << p1(0), p2(0);
        second_path_x = ptemp1;
//        second_path_x(1) = p2(0);
        ptemp2 << p1(1), p2(1);
        second_path_y = ptemp2;
//        second_path_y(1) = p2(1);
    } else if( bestTrajType[1] == 'r' ) {
        // Second part of teh path is a right turn
        phi_start = atan2( lrl_lr_tangent(1)-lrl_r_circle_center(1),
                           lrl_lr_tangent(0)-lrl_r_circle_center(0) );
        phi_end = atan2( lrl_rl_tangent(1)-lrl_r_circle_center(1),
                         lrl_rl_tangent(0)-lrl_r_circle_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val >= phi_end ) {
                phis(i) = val;
                val -= delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        second_path_x = lrl_r_circle_center(0) + r_min*phis.cos();
        second_path_y = lrl_r_circle_center(1) + r_min*phis.sin();
    } else if( bestTrajType[1] == 'l' ) {
        // Second part of the path is a left turn
        phi_start = atan2( rlr_rl_tangent(1)-rlr_l_circle_center(1),
                           rlr_rl_tangent(0)-rlr_l_circle_center(0) );
        phi_end = atan2( rlr_lr_tangent(1)-rlr_l_circle_center(1),
                         rlr_lr_tangent(0)-rlr_l_circle_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val <= phi_end ) {
                phis(i) = val;
                val += delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        second_path_x = rlr_l_circle_center(0) + r_min*phis.cos();
        second_path_y = rlr_l_circle_center(1) + r_min*phis.sin();
    }


    // Calculate the third part of the path
    if( bestTrajType[2] == 'r' ) {
        // Third part of path is a right hand turn
        if( bestTrajType == "rsr" ) {
            p(0) = rsr_tangent_x(1);
            p(1) = rsr_tangent_y(1);
        } else if( bestTrajType == "lsr" ) {
            p(0) = lsr_tangent_x(1);
            p(1) = lsr_tangent_y(1);
        } else { // bestTrajType == "rlr"
            p = rlr_lr_tangent;
        }

        phi_start = atan2( p(1)-grc_center(1), p(0)-grc_center(0) );
        phi_end = atan2( goal_location(1)-grc_center(1),
                         goal_location(0)-grc_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val >= phi_end ) {
                phis(i) = val;
                val -= delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        third_path_x = grc_center(0) + r_min*phis.cos();
        third_path_y = grc_center(1) + r_min*phis.sin();
    } else if( bestTrajType[2] == 'l' ) {
        // Third part of path is a left hand turn
        if( bestTrajType == "lsl" ) {
            p(0) = lsl_tangent_x(1);
            p(1) = lsl_tangent_y(1);
        } else if( bestTrajType == "rsl" ) {
            p(0) = rsl_tangent_x(1);
            p(1) = rsl_tangent_y(1);
        } else { // bestTrajType = "lrl"
            p = lrl_rl_tangent;
        }

        phi_start = atan2( p(1)-glc_center(1), p(0)-glc_center(0) );
        phi_end = atan2( goal_location(1)-glc_center(1),
                         goal_location(0)-glc_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

        phis = Eigen::VectorXd::Zero(std::ceil(std::abs(phi_end-phi_start)
                                               / delta_phi)+1);

        if( phi_end == phi_start ) {
            phis(0) = phi_start;
        } else {
            val = phi_start;
            i = 0;
            while( val <= phi_end ) {
                phis(i) = val;
                val += delta_phi;
                i += 1;
            }
            phis(i) = phi_end;
        }

        third_path_x = glc_center(0) + r_min*phis.cos();
        third_path_y = glc_center(1) + r_min*phis.sin();
    }

    this->edge_type_ = bestTrajType;
    this->w_dist_ = bestDist; // distance that the robot moves in the workspace

    int trajLength = first_path_x.size() + second_path_x.size() + third_path_x.size();
    Eigen::VectorXd traj1(trajLength), traj2(trajLength);

    if( this->w_dist_ == INF ) {
        this->dist_ = INF;
    } else if( this->cspace_->space_has_time_ ) {
        // Calculate C-space edge length
        // Note this HARDCODED batch version of DubinsDistAlongPath only
        // works because we assume constant speed along the edge
        this->dist_ = sqrt(pow(bestDist,2)
                          + pow(this->start_node_->position_(2)
                                - this->end_node_->position_(2), 2) );

        /* We need to calculate the time parameterization for the robot
         * along the path. NOTE: We make the simplifying assumption that
         * the robot travels at constant velocity along the entire path.
         * FURTHERMORE, the total time that it requires to traverse the
         * trajectory is determined by the difference in time positions
         * of the end points. ALSO NOTE: This function is not responsible
         * for determining if it is possible for the robot to actually
         * achieve this speed -- which is done in the function ValidMove().
         * FINALLY, we allso assume that the trajectory is sampled well
         * enough in "curvy" parts that the distance between points can
         * be approximated by the straight line distance between these
         * points i.e. the sum of the segment lengths is close enough to
         * this->w_dist_ that problems will not occur.
         */

        this->velocity_ = this->w_dist_
                / (this->start_node_->position_(2)-this->end_node_->position_(2));

        // Build trajectory with 0 for times
        Eigen::VectorXd zeros(first_path_x.size()
                              + second_path_x.size()
                              + third_path_x.size());


        traj1.block(0,0, first_path_x.size(),1) = first_path_x;
        traj1.block(first_path_x.size(),0,
                    second_path_x.size(),1) = second_path_x;
        traj1.block(first_path_x.size()+second_path_x.size(),0,
                    third_path_x.size(),1) = third_path_x;

        traj2.block(0,0, first_path_y.size(),1) = first_path_y;
        traj2.block(first_path_y.size(),0,
                    second_path_y.size(),1) = second_path_y;
        traj2.block(first_path_y.size()+second_path_y.size(),0,
                    third_path_y.size(),1) = third_path_y;

        zeros = Eigen::VectorXd::Zero(first_path_x.size()
                                      + second_path_x.size()
                                      + third_path_x.size());


        this->trajectory_.block(0,0,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = traj1;
        this->trajectory_.block(0,1,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = traj2;
        this->trajectory_.block(0,2,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = zeros;

        // Now calculate times
        this->trajectory_(0,2) = this->start_node_->position_(2);
        double cumulativeDist = 0.0;
        for( int j = 1; j < this->trajectory_.rows()-1; j++ ) {
            cumulativeDist += this->tree_->distanceFunction(
                        this->trajectory_.row(j-1).head(2),
                        this->trajectory_.row(j).head(2));
            this->trajectory_(j,2) = this->start_node_->position_(2)
                    - cumulativeDist/this->velocity_;
        }
        this->trajectory_.row(this->trajectory_.rows()-1).head(3)
                = this->end_node_->position_.head(3); // make end point exact
    } else {
        this->dist_ = bestDist;

        traj1.block(0,0, first_path_x.size(),1) = first_path_x;
        traj1.block(first_path_x.size(),0,
                    second_path_x.size(),1) = second_path_x;
        traj1.block(first_path_x.size()+second_path_x.size(),0,
                    third_path_x.size(),1) = third_path_x;

        traj2.block(0,0, first_path_y.size(),1) = first_path_y;
        traj2.block(first_path_y.size(),0,
                    second_path_y.size(),1) = second_path_y;
        traj2.block(first_path_y.size()+second_path_y.size(),0,
                    third_path_y.size(),1) = third_path_y;


        this->trajectory_.block(0,0, first_path_x.size() + second_path_x.size()
                               + third_path_x.size(),1) = traj1;
        this->trajectory_.block(0,1, first_path_x.size() + second_path_x.size()
                               + third_path_x.size(),1) = traj2;
    }

    this->dist_original_ = this->dist_;
}

void DubinsEdge::CalculateHoverTrajectory()
{
    this->edge_type_ = "xxx";
    this->w_dist_ = 0.0;
    this->dist_ = 0.0;

    if( this->cspace_->space_has_time_ ) {
        this->velocity_ = this->cspace_->dubins_min_velocity_;
        this->trajectory_.row(0) = this->start_node_->position_.head(3);
        this->trajectory_.row(1) = this->end_node_->position_.head(3);
    } else {
        this->trajectory_.row(0) = this->start_node_->position_.head(2);
        this->trajectory_.row(1) = this->end_node_->position_.head(2);
    }
}

/////////////////////// Collision Checking Functions ///////////////////////

bool DubinsEdge::ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle)
{
    // We know that all points in the Dubin's trajectory are within
    // r*S->min_turn_radius_ of the 2D segment from startNode to endNode
    // Thus, the edge is safe if a robot with this additional radius can
    // traverse that segment
    if(!ExplicitEdgeCheck2D(obstacle,
                            this->start_node_->position_,
                            this->end_node_->position_,
                            this->cspace_->robot_radius_
                            + 2*this->cspace_->min_turn_radius_)) {
//        std::cout << "Returning false" << std::endl;
        return false;
    }

//    std::cout << "robot cannot traverse edge:\n" << this->start_node_->position_
//              << std::endl << "--" << std::endl << this->start_node_->position_ << std::endl;

    // If the segment was in conflict then we need to do the full check
    // check of the trajectory segments
    // Could be improved using a function that can check arcs of the
    // Dubin's path at once instead of just line segments stored in trajectory
//    std::chrono::time_point<std::chrono::high_resolution_clock> startTime
//            = std::chrono::high_resolution_clock::now();
    //double time_start = getTimeNs(startTime);
    //double time_end;
    int count = 0;
    for(int i = 1; i < this->trajectory_.rows(); i++) {
        if((this->trajectory_.row(i)(0) > 0.001 || this->trajectory_.row(i)(1) > 0.001
            || this->trajectory_.row(i)(0) < -0.001 || this->trajectory_.row(i)(1) < -0.001)
                && (this->trajectory_.row(i-1)(0) != this->trajectory_.row(i)(0)
                || this->trajectory_.row(i-1)(1) != this->trajectory_.row(i)(1))) {
//            std::cout << "row: " << i+1 << std::endl;
            count++;
            if(ExplicitEdgeCheck2D(obstacle,
                                   this->trajectory_.row(i-1),
                                   this->trajectory_.row(i),
                                   this->cspace_->robot_radius_)) {
//                std::cout << "specifically edge segment:" << endl;
//                std::cout << "trajectory_.row("<<i<<"):\n" << this->trajectory_.row(i) << std::endl;
//                std::cout << "trajectory_.row("<<i-1<<"):\n" << this->trajectory_.row(i-1) << std::endl;
                //time_end = getTimeNs(startTime);
//                if(true) std::cout << "Check path: " << count << ": "
//                                   << (time_end-time_start)/MICROSECOND
//                                   << std::endl;
//                std::cout << "Obstacle:\n" << obstacle->position_ << std::endl
//                          << "with radius: " << obstacle->radius_ << std::endl
//                          << "in collision with this edge:\n"
//                     << this->start_node_->position_ << "\n--\n"
//                     << this->end_node_->position_ << std::endl;
                std::shared_ptr<Edge> this_edge = this->GetPointer();
                this->cspace_->AddVizEdge(this_edge);
                return true;
            }
        }
    }
    //time_end = getTimeNs(startTime);
//    if(true) std::cout << "Check path: " << count << ": "
//                       << (time_end-time_start)/MICROSECOND << " ms" << std::endl;
    return false;
}
