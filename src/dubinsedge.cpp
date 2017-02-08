#include <DRRT/dubinsedge.h>
#include <DRRT/kdtree.h>

/////////////////////// Critical Functions ///////////////////////

/////////////////////// Edge Functions ///////////////////////

std::shared_ptr<Edge> Edge::newEdge(std::shared_ptr<KDTreeNode> startNode,
                                    std::shared_ptr<KDTreeNode> endNode)
{
    return std::make_shared<DubinsEdge>(startNode,endNode);
}

bool DubinsEdge::validMove(std::shared_ptr<CSpace> S)
{
    if( S->spaceHasTime ) {
        // Note that planning happens in reverse time. i.e. time = 0 is at
        // the root of the search tree, and thus the time of startNode must be
        // greater than the time of endNode
        return ((this->startNode->position(2) > this->endNode->position(2))
                && ((S->dubinsMinVelocity <= this->velocity)
                    && (this->velocity <= S->dubinsMaxVelocity)));
    }
    // if space does not have time then we assume that a move is always valid
    return true;
}

Eigen::VectorXd DubinsEdge::poseAtDistAlongEdge(double distAlongEdge)
{
    double distRemaining = distAlongEdge;
    if( this->trajectory.rows() < 2 || this->dist <= distAlongEdge ) {
        return this->endNode->position;
    }

    // Find the piece of trajectory that contains the point at the desired distance
    int i = 1;
    double thisDist = INF;
    bool timeInPath = (this->trajectory.cols() >= 3);
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

Eigen::VectorXd DubinsEdge::poseAtTimeAlongEdge(double timeAlongEdge)
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

    Eigen::VectorXd vec;
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

void DubinsEdge::calculateTrajectory(std::shared_ptr<CSpace> S,
                                     std::shared_ptr<KDTree> Tree)
{
    double r_min = S->minTurningRadius;

    Eigen::Vector2d initial_location = this->startNode->position.head(2);
    double initial_theta = this->startNode->position(3);
    Eigen::Vector2d goal_location = this->endNode->position.head(2);
    double goal_theta = this->endNode->position(3);

    // Calculate the center of the right-turn and left-turn circles
    // ... right-turn initial_location circle
    Eigen::Vector2d temp;
    Eigen::Vector2d temp2;
    temp(0) = cos(initial_theta-(PI/2.0));
    temp(1) = sin(initial_theta-(PI/2.0));
    // ... left-turn initial location circle
    Eigen::Vector2d irc_center = initial_location + r_min * temp;
    temp(0) = cos(initial_theta+(PI/2.0));
    temp(1) = sin(initial_theta+(PI/2.0));
    // ... right-turn goal_location circle
    Eigen::Vector2d ilc_center = initial_location + r_min * temp;
    temp(0) = cos(goal_theta-(PI/2.0));
    temp(1) = sin(goal_theta-(PI/2.0));
    // ...left_turn goal_location circle
    Eigen::Vector2d grc_center = goal_location + r_min * temp;
    temp(0) = cos(goal_theta+(PI/2.0));
    temp(1) = sin(goal_theta+(PI/2.0));
    Eigen::Vector2d glc_center = goal_location + r_min * temp;


    // Remember the best distance and associated type of trajectory
    double bestDist = INF;
    std::string bestTrajType = "xxx";

    Eigen::ArrayXd diff, temp3(2), t; // Arrays used for coefficient-wise
                                      // operations (.operator() in Julia)
    double D, R, sq, a, b, firstDist, secondDist, thirdDist;
    Eigen::VectorXd v; // temp, temp2 defined above

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
        firstDist = rightTurnDist( initial_location, temp, irc_center, r_min );
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        temp2(0) = rsl_tangent_x(0);
        temp2(1) = rsl_tangent_y(0);
        diff = temp - temp2;
        secondDist = sqrt((diff*diff).sum());
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        thirdDist = leftTurnDist( temp, goal_location, glc_center, r_min );

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
    firstDist = rightTurnDist( initial_location, temp, irc_center, r_min );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    temp2(0) = rsr_tangent_x(0);
    temp2(1) = rsr_tangent_y(0);
    diff = temp - temp2;
    secondDist = sqrt( (diff*diff).sum() );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    thirdDist = rightTurnDist( temp, goal_location, grc_center, r_min );

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

        firstDist = rightTurnDist( initial_location, rlr_rl_tangent,
                                   irc_center, r_min );
        secondDist = leftTurnDist( rlr_rl_tangent, rlr_lr_tangent,
                                   rlr_l_circle_center, r_min );
        thirdDist = rightTurnDist( rlr_lr_tangent, goal_location,
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
        firstDist = leftTurnDist( initial_location, temp, ilc_center, r_min );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        temp2(0) = lsr_tangent_x(0);
        temp2(1) = lsr_tangent_y(0);
        diff = temp - temp2;
        secondDist = sqrt( (diff*diff).sum() );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        thirdDist = rightTurnDist( temp, goal_location, grc_center, r_min );

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
    firstDist = leftTurnDist( initial_location, temp, ilc_center, r_min );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    temp2(0) = lsl_tangent_x(0);
    temp2(1) = lsl_tangent_y(0);
    diff = temp - temp2;
    secondDist = sqrt( (diff*diff).sum() );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    thirdDist = leftTurnDist( temp, goal_location, glc_center, r_min );

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

        firstDist = rightTurnDist( initial_location, lrl_lr_tangent,
                                   ilc_center, r_min );
        secondDist = leftTurnDist( lrl_lr_tangent, lrl_rl_tangent,
                                   lrl_r_circle_center, r_min );
        thirdDist = rightTurnDist( lrl_rl_tangent, goal_location,
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

    this->edgeType = bestTrajType;
    this->Wdist = bestDist; // distance that the robot moves in the workspace

    int trajLength = first_path_x.size() + second_path_x.size() + third_path_x.size();
    Eigen::VectorXd traj1(trajLength), traj2(trajLength);

    if( this->Wdist == INF ) {
        this->dist = INF;
    } else if( S->spaceHasTime ) {
        // Calculate C-space edge length
        // Note this HARDCODED batch version of dubinsDistAlongPath only
        // works because we assume constant speed along the edge
        this->dist = sqrt(pow(bestDist,2)
                          + pow(this->startNode->position(2)
                                - this->endNode->position(2), 2) );

        /* We need to calculate the time parameterization for the robot
         * along the path. NOTE: We make the simplifying assumption that
         * the robot travels at constant velocity along the entire path.
         * FURTHERMORE, the total time that it requires to traverse the
         * trajectory is determined by the difference in time positions
         * of the end points. ALSO NOTE: This function is not responsible
         * for determining if it is possible for the robot to actually
         * achieve this speed -- which is done in the function validMove().
         * FINALLY, we allso assume that the trajectory is sampled well
         * enough in "curvy" parts that the distance between points can
         * be approximated by the straight line distance between these
         * points i.e. the sum of the segment lengths is close enough to
         * this->Wdist that problems will not occur.
         */

        this->velocity = this->Wdist
                / (this->startNode->position(2)-this->endNode->position(2));

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


        this->trajectory.block(0,0,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = traj1;
        this->trajectory.block(0,1,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = traj2;
        this->trajectory.block(0,2,
            first_path_x.size() + second_path_x.size() + third_path_x.size(),
                               1) = zeros;

        // Now calculate times
        this->trajectory(0,2) = this->startNode->position(2);
        double cumulativeDist = 0.0;
        for( int j = 1; j < this->trajectory.rows()-1; j++ ) {
            cumulativeDist += Tree->distanceFunction(
                        this->trajectory.row(j-1).head(2),
                        this->trajectory.row(j).head(2));
            this->trajectory(j,2) = this->startNode->position(2)
                    - cumulativeDist/this->velocity;
        }
        this->trajectory.row(this->trajectory.rows()-1).head(3)
                = this->endNode->position.head(3); // make end point exact
    } else {
        this->dist = bestDist;

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


        this->trajectory.block(0,0, first_path_x.size() + second_path_x.size()
                               + third_path_x.size(),1) = traj1;
        this->trajectory.block(0,1, first_path_x.size() + second_path_x.size()
                               + third_path_x.size(),1) = traj2;
    }

    this->distOriginal = this->dist;
}

void DubinsEdge::calculateHoverTrajectory(std::shared_ptr<CSpace> S)
{
    this->edgeType = "xxx";
    this->Wdist = 0.0;
    this->dist = 0.0;

    if( S->spaceHasTime ) {
        this->velocity = S->dubinsMinVelocity;
        this->trajectory.row(0) = this->startNode->position.head(3);
        this->trajectory.row(1) = this->endNode->position.head(3);
    } else {
        this->trajectory.row(0) = this->startNode->position.head(2);
        this->trajectory.row(1) = this->endNode->position.head(2);
    }
}

/////////////////////// Collision Checking Functions ///////////////////////

//bool Edge::explicitEdgeCheck( std::shared_ptr<CSpace> S, Obstacle* obstacle ){}
