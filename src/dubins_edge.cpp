#include <DRRT/dubins_edge.h>
#include <DRRT/kdnode.h>
#include <DRRT/data_structures.h>

Edge_ptr Edge::NewEdge(Kdnode_ptr start, Kdnode_ptr end)
{
    Edge_ptr new_edge = std::make_shared<DubinsEdge>(start, end);
    new_edge->SetDist(DistanceFunction(start->GetPosition(), end->GetPosition()));
    return new_edge;
}

Edge_ptr Edge::NewEdge()
{
    Edge_ptr new_edge = std::make_shared<DubinsEdge>(); // new_edge->dist_ == INF
    return new_edge;
}

bool DubinsEdge::ValidMove()
{
    return true;
}

Eigen::VectorXd DubinsEdge::PoseAtDistAlongEdge(double dist_along_edge)
{
    double dist_remaining = dist_along_edge;
    if(GetTrajectory().rows() < 2 || GetDist() <= dist_remaining)
        return GetEnd()->GetPosition();

    // Find the piece of the trajectory that contains the point
    int i = 1;
    double dist = INF;
    while(i <= GetTrajectory().rows()) {
        dist = DubinsDistance(GetTrajectory().row(i-1), GetTrajectory().row(i));
        if(dist_remaining <= dist) break;
        dist_remaining -= dist;
        i += 1;
    }

    // For subtraction-based precision errors
    if(dist_remaining > dist) dist_remaining = dist;

    // Calculate the pose along that path
    double ratio = dist_remaining / dist;
    Eigen::VectorXd ret = GetTrajectory().row(i-1)
            + ratio*(GetTrajectory().row(i) - GetTrajectory().row(i-1));
    double ret_theta = atan2(GetTrajectory()(i,1) - GetTrajectory()(i-1,1),
                             GetTrajectory()(i,0) - GetTrajectory()(i-1,0));

    Eigen::VectorXd vec;
    vec.resize(NUM_DIM);

    // TODO: Generalize this to other spaces besides Dubins
    vec(0) = ret(0);  // x-coordinate
    vec(1) = ret(1);  // y-coordinate
    vec(2) = ret_theta;
    return vec;
}

void DubinsEdge::CalculateTrajectory(CSpace_ptr cspace)
{
    double r_min = cspace->min_turn_radius_;

    Eigen::Vector2d initial_location = GetStart()->GetPosition().head(2);
    double initial_theta = GetStart()->GetPosition()(2);
    Eigen::Vector2d goal_location = GetEnd()->GetPosition().head(2);
    double goal_theta = GetEnd()->GetPosition()(2);

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
    double best_dist = INF;
    std::string best_traj_type = "xxx";

    Eigen::ArrayXd diff, temp3(2), t; // Arrays used for coefficient-wise
                                      // operations
    double D, R, sq, a, b, first_dist, second_dist, third_dist;
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
        first_dist = RightTurnDist( initial_location, temp, irc_center, r_min );
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        temp2(0) = rsl_tangent_x(0);
        temp2(1) = rsl_tangent_y(0);
        diff = temp - temp2;
        second_dist = sqrt((diff*diff).sum());
        temp(0) = rsl_tangent_x(1);
        temp(1) = rsl_tangent_y(1);
        third_dist = LeftTurnDist( temp, goal_location, glc_center, r_min );

        rsl_length = first_dist + second_dist + third_dist;

        if( best_dist > rsl_length ) {
            best_dist = rsl_length;
            best_traj_type = "rsl";
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
    first_dist = RightTurnDist( initial_location, temp, irc_center, r_min );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    temp2(0) = rsr_tangent_x(0);
    temp2(1) = rsr_tangent_y(0);
    diff = temp - temp2;
    second_dist = sqrt( (diff*diff).sum() );
    temp(0) = rsr_tangent_x(1);
    temp(1) = rsr_tangent_y(1);
    third_dist = RightTurnDist( temp, goal_location, grc_center, r_min );

    rsr_length = first_dist + second_dist + third_dist;

    if( best_dist > rsr_length ) {
        best_dist = rsr_length;
        best_traj_type = "rsr";
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

        first_dist = RightTurnDist( initial_location, rlr_rl_tangent,
                                   irc_center, r_min );
        second_dist = LeftTurnDist( rlr_rl_tangent, rlr_lr_tangent,
                                   rlr_l_circle_center, r_min );
        third_dist = RightTurnDist( rlr_lr_tangent, goal_location,
                                   grc_center, r_min );

        rlr_length = first_dist + second_dist + third_dist;

        if( best_dist > rlr_length ) {
            best_dist = rlr_length;
            best_traj_type = "rlr";
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
        first_dist = LeftTurnDist( initial_location, temp, ilc_center, r_min );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        temp2(0) = lsr_tangent_x(0);
        temp2(1) = lsr_tangent_y(0);
        diff = temp - temp2;
        second_dist = sqrt( (diff*diff).sum() );
        temp(0) = lsr_tangent_x(1);
        temp(1) = lsr_tangent_y(1);
        third_dist = RightTurnDist( temp, goal_location, grc_center, r_min );

        lsr_length = first_dist + second_dist + third_dist;

        if( best_dist > lsr_length ) {
            best_dist = lsr_length;
            best_traj_type = "lsr";
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
    first_dist = LeftTurnDist( initial_location, temp, ilc_center, r_min );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    temp2(0) = lsl_tangent_x(0);
    temp2(1) = lsl_tangent_y(0);
    diff = temp - temp2;
    second_dist = sqrt( (diff*diff).sum() );
    temp(0) = lsl_tangent_x(1);
    temp(1) = lsl_tangent_y(1);
    third_dist = LeftTurnDist( temp, goal_location, glc_center, r_min );

    lsl_length = first_dist + second_dist + third_dist;

    if( best_dist > lsl_length ) {
        best_dist = lsl_length;
        best_traj_type = "lsl";
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

        first_dist = RightTurnDist( initial_location, lrl_lr_tangent,
                                   ilc_center, r_min );
        second_dist = LeftTurnDist( lrl_lr_tangent, lrl_rl_tangent,
                                   lrl_r_circle_center, r_min );
        third_dist = RightTurnDist( lrl_rl_tangent, goal_location,
                                   glc_center, r_min );

        lrl_length = first_dist + second_dist + third_dist;

        if( best_dist > lrl_length ) {
            best_dist = lrl_length;
            best_traj_type = "lrl";
        } else {
            lrl_length = INF;
        }
    }

    /// FOR DEBUGGING BULLET
//    if(this->dist_ <= 2.0
//            && this->start_node_->position_(2)
//            == this->end_node_->position_(2))
//        best_traj_type = "0s0";

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
    if( best_traj_type[0] == 'r' ) {
        // First part of the path is a right hand turn
        if( best_traj_type == "rsl" ) {
            p(0) = rsl_tangent_x(0);
            p(1) = rsl_tangent_y(0);
        } else if( best_traj_type == "rsr" ) {
            p(0) = rsr_tangent_x(0);
            p(1) = rsr_tangent_y(0);
        } else { // best_traj_type == "rlr"
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
    } else if( best_traj_type[0] == 'l' ) {
        // First part of the path is a left hand turn
        if( best_traj_type == "lsl" ) {
            p(0) = lsl_tangent_x(0);
            p(1) = lsl_tangent_y(0);
        } else if( best_traj_type == "lsr" ) {
            p(0) = lsr_tangent_x(0);
            p(1) = lsr_tangent_y(0);
        } else { // best_traj_type == "lrl"
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
    if( best_traj_type[1] == 's' ) {
        // Second part of the path is a straight line
        if( best_traj_type == "lsr" ) {
            p1(0) = lsr_tangent_x(0);
            p1(1) = lsr_tangent_y(0);
            p2(0) = lsr_tangent_x(1);
            p2(1) = lsr_tangent_y(1);
        } else if( best_traj_type == "lsl" ) {
            p1(0) = lsl_tangent_x(0);
            p1(1) = lsl_tangent_y(0);
            p2(0) = lsl_tangent_x(1);
            p2(1) = lsl_tangent_y(1);
        } else if( best_traj_type == "rsr" ) {
            p1(0) = rsr_tangent_x(0);
            p1(1) = rsr_tangent_y(0);
            p2(0) = rsr_tangent_x(1);
            p2(1) = rsr_tangent_y(1);
        } else if( best_traj_type == "rsl" ) {
            p1(0) = rsl_tangent_x(0);
            p1(1) = rsl_tangent_y(0);
            p2(0) = rsl_tangent_x(1);
            p2(1) = rsl_tangent_y(1);
        } else { // best_traj_type = "0s0"
            p1(0) = GetStart()->GetPosition()(0);
            p1(1) = GetStart()->GetPosition()(1);
            p2(0) = GetEnd()->GetPosition()(0);
            p2(1) = GetEnd()->GetPosition()(1);
        }

        Eigen::Vector2d ptemp1, ptemp2;

        ptemp1 << p1(0), p2(0);
        second_path_x = ptemp1;
//        second_path_x(1) = p2(0);
        ptemp2 << p1(1), p2(1);
        second_path_y = ptemp2;
//        second_path_y(1) = p2(1);
    } else if( best_traj_type[1] == 'r' ) {
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
    } else if( best_traj_type[1] == 'l' ) {
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
    if( best_traj_type[2] == 'r' ) {
        // Third part of path is a right hand turn
        if( best_traj_type == "rsr" ) {
            p(0) = rsr_tangent_x(1);
            p(1) = rsr_tangent_y(1);
        } else if( best_traj_type == "lsr" ) {
            p(0) = lsr_tangent_x(1);
            p(1) = lsr_tangent_y(1);
        } else { // best_traj_type == "rlr"
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
    } else if( best_traj_type[2] == 'l' ) {
        // Third part of path is a left hand turn
        if( best_traj_type == "lsl" ) {
            p(0) = lsl_tangent_x(1);
            p(1) = lsl_tangent_y(1);
        } else if( best_traj_type == "rsl" ) {
            p(0) = rsl_tangent_x(1);
            p(1) = rsl_tangent_y(1);
        } else { // best_traj_type = "lrl"
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

    SetType(best_traj_type);

    int traj_length = first_path_x.size()
                    + second_path_x.size()
                    + third_path_x.size();
    Eigen::VectorXd traj1(traj_length), traj2(traj_length);
    Eigen::MatrixXd traj;

    // Dubins model has 3 columns [x, y, theta] set on creation of edge trajectory
    // However trajectory only takes into account theta at nodes and not on
    // trajectories between them. Theta can be assumed to be the tangent line to
    // the circle used to create the path
    traj.resize(traj_length, NUM_DIM);

    SetDist(best_dist);

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

    traj.block(0,0, traj_length,1) = traj1;
    traj.block(0,1, traj_length,1) = traj2;

    SetTrajectory(traj);
}

void DubinsEdge::CalculateHoverTrajectory()
{
    SetType("xxx");
    SetDist(0.0);
    Eigen::MatrixX2d traj;
    traj.resize(2, Eigen::NoChange_t());
    traj.row(0) = GetStart()->GetPosition().head(2);
    traj.row(1) = GetStart()->GetPosition().head(2);
    SetTrajectory(traj);
}
