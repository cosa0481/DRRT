/* edge.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/kdtree.h>
#include <DRRT/edge.h>

#define PI 3.1415926536

/////////////////////// Critical Functions ///////////////////////

double Edist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    return distFunc( "R3SDist", x, y );
}

double EKDdist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    return distFunc( "EuclidianDist", x, y );
}

double EWdist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    return distFunc( "EuclidianDist", x.head(2), y.head(2) );
}

void saturate( Eigen::VectorXd newPoint, Eigen::VectorXd closestPoint,
               double delta )
{
    double thisDist = Edist( newPoint, closestPoint );
    if( thisDist > delta ) {
        // First scale non-theta dimensions
        newPoint.head(3) = closestPoint.head(3) +
                ( newPoint.head(3) - closestPoint.head(3) ) * delta / thisDist;

        // Saturate theta in the short of the two directions that it can go
        if( std::abs( newPoint(3) - closestPoint(3) ) < PI ) {
            // Saturate in the normal way
            newPoint(3) = closestPoint(3) +
                    (newPoint(3) - closestPoint(3)) * delta / thisDist;
        } else {
            // Saturate in the opposite way
            if( newPoint(3) < PI ) {
                newPoint(3) = newPoint(3) + 2*PI;
            } else {
                newPoint(3) = newPoint(3) - 2*PI;
            }

            // Now saturate
            newPoint(3) = closestPoint(3) +
                    (newPoint(3) - closestPoint(3)) * delta / thisDist;

            // Finally, wrpap back to the identity that is on [0 2pi]
            // Is this really wrapping?

            Eigen::VectorXd minVec;
            minVec(0) = newPoint(3);
            minVec(1) = 2*PI;
            Eigen::VectorXd maxVec;
            maxVec(0) = minVec.minCoeff();
            maxVec(1) = 0.0;
            newPoint(3) = maxVec.maxCoeff();
        }
    }
}


/////////////////////// Edge Functions ///////////////////////

Edge* newEdge( KDTreeNode* startNode, KDTreeNode* endNode )
{
    return new Edge( startNode, endNode );
}

bool validMove( CSpace* S, Edge* edge )
{
    if( S->spaceHasTime ) {
        // Note that planning happens in reverse time. i.e. time = 0 is at
        // the root of the search tree, and thus the time of startNode must be
        // greater than the time of endNode
        return ((edge->startNode->position(2) > edge->endNode->position(2)) && ((S->dubinsMinVelocity <= edge->velocity) && (edge->velocity <= S->dubinsMaxVelocity)));
    }
    // if space does not have time then we assume that a move is always valid
    return true;
}

Eigen::VectorXd poseAtDistAlongEdge( Edge* edge, double distAlongEdge )
{
    double distRemaining = distAlongEdge;
    if( edge->trajectory.rows() < 2 || edge->dist <= distAlongEdge ) {
        return edge->endNode->position;
    }

    // Find the piece of trajectory that contains the point at the desired distance
    int i = 1;
    double thisDist = INF;
    bool timeInPath = (edge->trajectory.cols() >= 3);
    while( i <= edge->trajectory.rows() ) {
        double wtime = dubinsDistAlongTimePath( edge->trajectory.row(i-1), edge->trajectory.row(i) );
        double wotime = dubinsDistAlongPath( edge->trajectory.row(i-1), edge->trajectory.row(i) );
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
    Eigen::VectorXd ret = edge->trajectory.row(i-1) + ratio*(edge->trajectory.row(i)-edge->trajectory.row(i-1));
    double retTimeRatio = distAlongEdge/edge->dist;
    double retTime = edge->startNode->position(2) + retTimeRatio*(edge->endNode->position(2) - edge->startNode->position(2));
    double retTheta = atan2 (edge->trajectory(i,1) - edge->trajectory(i-1,1), edge->trajectory(i,0) - edge->trajectory(i-1,0) );

    Eigen::VectorXd vec;
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

Eigen::VectorXd poseAtTimeAlongEdge( Edge* edge, double timeAlongEdge )
{
    if( edge->trajectory.rows() < 2 || (edge->startNode->position(2) - edge->endNode->position(2)) <= timeAlongEdge ) {
        return edge->endNode->position;
    }

    // Find the piece of the trajectory that contains the time at the
    // desired distance
    int i = 1;
    while( edge->trajectory(i,2) > edge->startNode->position(2) - timeAlongEdge ) {
        i += 1;
    }

    // Now calculate pose along that piece
    double ratio = (edge->trajectory(i-1,2) - (edge->startNode->position(2)-timeAlongEdge)) / (edge->trajectory(i-1,2) - edge->trajectory(i,2));
    Eigen::VectorXd ret = edge->trajectory.row(i-1) + ratio*(edge->trajectory.row(i) - edge->trajectory.row(i-1));
    double retTime = edge->startNode->position(2) - timeAlongEdge;
    double retTheta = atan2( edge->trajectory(i,1) - edge->trajectory(i-1,1), edge->trajectory(i,0) - edge->trajectory(i-1,0) );

    Eigen::VectorXd vec;
    vec(0) = ret(0); // x-coordinate
    vec(1) = ret(1); // y-coordinate
    vec(2) = retTime;
    vec(3) = retTheta;

    return vec;
}

void calculateTrajectory( CSpace* S, Edge* edge )
{
    double r_min = S->minTurningRadius;

    Eigen::VectorXd initial_location = edge->startNode->position.head(2);
    double initial_theta = edge->startNode->position(3);
    Eigen::VectorXd goal_location = edge->endNode->position.head(2);
    double goal_theta = edge->endNode->position(3);

    // Calculate the center of the right-turn and left-turn circles
    // ... right-turn initial_location circle
    Eigen::VectorXd temp;
    Eigen::VectorXd temp2;
    temp(0) = cos(initial_theta-(PI/2.0));
    temp(1) = sin(initial_theta-(PI/2.0));
    // ... left-turn initial location circle
    Eigen::VectorXd irc_center = initial_location + r_min * temp;
    temp(0) = cos(initial_theta+(PI/2.0));
    temp(1) = sin(initial_theta+(PI/2.0));
    // ... right-turn goal_location circle
    Eigen::VectorXd ilc_center = initial_location + r_min * temp;
    temp(0) = cos(goal_theta-(PI/2.0));
    temp(1) = sin(goal_theta-(PI/2.0));
    // ...left_turn goal_location circle
    Eigen::VectorXd grc_center = goal_location + r_min * temp;
    temp(0) = cos(goal_theta+(PI/2.0));
    temp(1) = sin(goal_theta+(PI/2.0));
    Eigen::VectorXd glc_center = goal_location + r_min * temp;

    // Remember the best distance and associated type of trajectory
    double bestDist = INF;
    std::string bestTrajType = "xxx";

    Eigen::ArrayXd diff, temp3, t; // Arrays used for coefficient-wise operations (.operator() in Julia)
    double D, R, sq, a, b, firstDist, secondDist, thirdDist;
    Eigen::VectorXd v; // temp, temp2 defined above


    // Calculate tangent points for right-straight-left path
    // r-s-l requires "inner" tangent points
    diff = glc_center - irc_center;
    D = sqrt((diff*diff).sum());
    v = diff/D;
    R = -2.0*r_min/D;
    double rsl_length;
    Eigen::VectorXd rsl_tangent_x, rsl_tangent_y;
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
    Eigen::VectorXd  rsr_tangent_x, rsr_tangent_y;
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
    Eigen::ArrayXd rlr_rl_tangent;
    Eigen::ArrayXd rlr_lr_tangent;
    rlr_rl_tangent(0) = 0;
    rlr_rl_tangent(1) = 0;
    rlr_lr_tangent(0) = 0;
    rlr_lr_tangent(1) = 0;

    // D was last set by the rsr calculation
    Eigen::VectorXd rlr_l_circle_center;
    if( D < 4.0*r_min ) {
        // Start by finding the center of the "left" circle
        double theta = -acos(D/(4*r_min)) + atan2(v(1),v(0));
        temp3(0) = cos(theta);
        temp3(1) = sin(theta);
        t = irc_center;
        rlr_l_circle_center = t + 2*r_min*temp3;
        rlr_rl_tangent = (rlr_l_circle_center + irc_center)/2.0;
        rlr_lr_tangent = (rlr_l_circle_center + grc_center)/2.0;

        firstDist = rightTurnDist( initial_location, rlr_rl_tangent, irc_center, r_min );
        secondDist = leftTurnDist( rlr_rl_tangent, rlr_lr_tangent, rlr_l_circle_center, r_min );
        thirdDist = rightTurnDist( rlr_lr_tangent, goal_location, grc_center, r_min );

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
    Eigen::VectorXd lsr_tangent_x, lsr_tangent_y;
    if( std::abs(R) > 1.0 ) {
        lsr_length = INF;
    } else {
        sq = sqrt(1.0-R*R);
        a = R*v(0) + v(1)*sq;
        b = R*v(1) - v(0)*sq;
        lsr_tangent_x(0) = ilc_center(0) + a*r_min;
        lsr_tangent_x(1) = grc_center(0) + a*r_min;
        lsr_tangent_y(0) = ilc_center(1) + b*r_min;
        lsr_tangent_y(1) = grc_center(1) + b*r_min;

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
    Eigen::VectorXd lsl_tangent_x, lsl_tangent_y;
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
    Eigen::ArrayXd lrl_rl_tangent, lrl_lr_tangent;
    lrl_rl_tangent(0) = 0;
    lrl_rl_tangent(1) = 0;
    lrl_lr_tangent(0) = 0;
    lrl_lr_tangent(1) = 0;

    // D was last set by the lsl calculation
    Eigen::VectorXd lrl_r_circle_center;
    if( D < 4.0*r_min ) {
        // Start by finding the center of the "left" circle
        double theta = -acos(D/(4*r_min)) + atan2(v(1),v(0));
        temp3(0) = cos(theta);
        temp3(1) = sin(theta);
        t = ilc_center;
        lrl_r_circle_center = t + 2*r_min*temp3;
        lrl_lr_tangent = (lrl_r_circle_center + ilc_center)/2.0;
        lrl_rl_tangent = (lrl_r_circle_center + glc_center)/2.0;

        firstDist = rightTurnDist( initial_location, lrl_lr_tangent, ilc_center, r_min );
        secondDist = leftTurnDist( lrl_lr_tangent, lrl_rl_tangent, lrl_r_circle_center, r_min );
        thirdDist = rightTurnDist( lrl_rl_tangent, goal_location, glc_center, r_min );

        lrl_length = firstDist + secondDist + thirdDist;

        if( bestDist > lrl_length ) {
            bestDist = lrl_length;
            bestTrajType = "lrl";
        } else {
            lrl_length = INF;
        }
    }


    // Now save the best path in the trajectory field

    double delta_phi = 0.1; // this is the angle granularity (in radians)
                            // used for discritizing the arcs of the paths
                            // (straight lines are saved as a single segment

    Eigen::ArrayXd phis;    // This array contains 1 element or a sequence of elements
    Eigen::VectorXd p, p1, p2, first_path_x, first_path_y, second_path_x, second_path_y, third_path_x, third_path_y;
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

        phi_start = atan2( initial_location(1)-irc_center(1), initial_location(0)-irc_center(0) );
        phi_end = atan2( p(1)-irc_center(1), p(0)-irc_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

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

        phi_start = atan2( initial_location(1)-ilc_center(1), initial_location(0)-ilc_center(0) );
        phi_end = atan2( p(1)-ilc_center(1), p(0)-ilc_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

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

        second_path_x(0) = p1(0);
        second_path_x(1) = p2(0);
        second_path_y(0) = p1(1);
        second_path_y(1) = p2(1);
    } else if( bestTrajType[1] == 'r' ) {
        // Second part of teh path is a right turn
        phi_start = atan2( lrl_lr_tangent(1)-lrl_r_circle_center(1), lrl_lr_tangent(0)-lrl_r_circle_center(0) );
        phi_end = atan2( lrl_rl_tangent(1)-lrl_r_circle_center(1), lrl_rl_tangent(0)-lrl_r_circle_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

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
        phi_start = atan2( rlr_rl_tangent(1)-rlr_l_circle_center(1), rlr_rl_tangent(0)-rlr_l_circle_center(0) );
        phi_end = atan2( rlr_lr_tangent(1)-rlr_l_circle_center(1), rlr_lr_tangent(0)-rlr_l_circle_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

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
        phi_end = atan2( goal_location(1)-grc_center(1), goal_location(0)-grc_center(0) );

        if( phi_end > phi_start ) {
            phi_end -= 2.0*PI;
        }

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
        phi_end = atan2( goal_location(1)-glc_center(1), goal_location(0)-glc_center(0) );

        if( phi_end < phi_start ) {
            phi_end += 2.0*PI;
        }

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

    edge->edgeType = bestTrajType;
    edge->Wdist = bestDist;    // distance that the robot moves in the workspace

    if( edge->Wdist == INF ) {
        edge->dist = INF;
    } else if( S->spaceHasTime ) {
        // Calculate C-space edge length
        // Note this HARDCODED batch version of dubinsDistAlongPath only
        // works because we assume constant speed along the edge
        edge->dist = sqrt(pow(bestDist,2) + pow(edge->startNode->position(2)-edge->endNode->position(2), 2) );

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
         * edge->Wdist that problems will not occur.
         */

        edge->velocity = edge->Wdist/(edge->startNode->position(2)-edge->endNode->position(2));

        // Build trajectory with 0 for times
        Eigen::VectorXd traj1, traj2, traj3;
        traj1 << first_path_x , first_path_y , Eigen::VectorXd::Zero(first_path_x.size());
        traj2 << second_path_x , second_path_y , Eigen::VectorXd::Zero(second_path_x.size());
        traj3 << third_path_x , third_path_y , Eigen::VectorXd::Zero(third_path_x.size());

        edge->trajectory.row(0) = traj1;
        edge->trajectory.row(1) = traj2;
        edge->trajectory.row(2) = traj3;

        // Now calculate times
        edge->trajectory(0,2) = edge->startNode->position(2);
        double cumulativeDist = 0.0;
        for( int j = 1; j < edge->trajectory.rows()-1; j++ ) {
            cumulativeDist += EWdist( edge->trajectory.row(j-1).head(2), edge->trajectory.row(j).head(2) );
            edge->trajectory(j,2) = edge->startNode->position(2) - cumulativeDist/edge->velocity;
        }
        edge->trajectory.row(edge->trajectory.rows()-1).head(3) = edge->endNode->position.head(3); // make end point exact
    } else {
        edge->dist = bestDist;
        Eigen::VectorXd traj11, traj22, traj33;
        traj11 << first_path_x, first_path_y;
        traj22 << second_path_x, second_path_y;
        traj33 << third_path_x, third_path_y;

        edge->trajectory.row(0) = traj11;
        edge->trajectory.row(1) = traj22;
        edge->trajectory.row(2) = traj33;
    }

    edge->distOriginal = edge->dist;
}

void calculateHoverTrajectory( CSpace* S, Edge* edge )
{
    edge->edgeType = "xxx";
    edge->Wdist = 0.0;
    edge->dist = 0.0;

    if( S->spaceHasTime ) {
        edge->velocity = S->dubinsMinVelocity;
        edge->trajectory.row(0) = edge->startNode->position.head(3);
        edge->trajectory.row(1) = edge->endNode->position.head(3);
    } else {
        edge->trajectory.row(0) = edge->startNode->position.head(2);
        edge->trajectory.row(1) = edge->endNode->position.head(2);
    }
}


/////////////////////// Collision Checking Functions ///////////////////////

//void explicitEdgeCheck( CSpace* S, Edge* edge, Obstacle* obstacle ) {}
