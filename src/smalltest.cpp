/* smalltest.cpp
 * Corin Sandford
 * Spring 2017
 * Used to test high level things in the
 * library on a smaller scale than test.cpp
 */

#include <DRRT/drrt.h>
#include <DRRT/visualizer.h>
#include <DRRT/theta_star.h>

using namespace std;

bool timingex = false;

chrono::time_point<chrono::high_resolution_clock> startTime;

void PrintRrtxPath(shared_ptr<KDTreeNode> &leaf)
{
    std::cout << "\nRRTx Path" << std::endl;
    while(leaf->rrtParentUsed) {
        cout << "pose: " << leaf->rrtLMC << "\n" << leaf->position << endl;
        cout << "VVVVVVVV" << endl;
        leaf = leaf->rrtParentEdge->end_node_;
    }
    cout << leaf->position << endl;
}

/// AlGORITHM CONTROL FUNCTION
// This function runs RRTx with the parameters defined in main()
shared_ptr<RobotData> Rrtx(Problem p, shared_ptr<thread> &vis)
{
    // Used for "sensing" obstacles (should make input, from DRRT.jl)
    double robot_sensor_range = 20.0;

    /// Initialize

    /// Queue
    shared_ptr<Queue> Q = make_shared<Queue>();
    Q->priority_queue = make_shared<BinaryHeap>(false); // priority queue use Q functions
    Q->obs_successors = make_shared<JList>(true); // obstacle stack uses KDTreeNodes
    Q->change_thresh = p.change_threshold;
    Q->type = p.search_type;
    Q->cspace = p.c_space;
    Q->cspace->sample_stack_ = make_shared<JList>(true); // uses KDTreeNodes
    Q->cspace->saturation_delta_ = p.delta;

    /// K-D Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(Q->cspace->num_dimensions_,p.wraps,p.wrap_points);
    kd_tree->setDistanceFunction(Q->cspace->distanceFunction);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->cspace->start_);
    ExplicitNodeCheck(Q,root);
    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;
    root->rrtParentEdge = Edge::NewEdge(Q->cspace,kd_tree,root,root);
    root->rrtParentUsed = false;
    kd_tree->kdInsert(root);

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->cspace->goal_);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;

    Q->cspace->goal_node_ = goal;
    Q->cspace->root_ = root;
    Q->cspace->move_goal_ = goal;
    Q->cspace->move_goal_->isMoveGoal = true;

    /// Robot
    shared_ptr<RobotData> robot
            = make_shared<RobotData>(Q->cspace->goal_, goal,
                                     MAXPATHNODES, Q->cspace->num_dimensions_);

    if(Q->cspace->space_has_time_) {
        addOtherTimesToRoot(Q->cspace,kd_tree,goal,root,Q->type);
    }

    // Store and print the path
    // Path in form -b*y = a*x + c
    vector<Eigen::Vector3d> lines;
    double angle, x = 0, y = 0;
    vector<Eigen::VectorXd> path = ThetaStar(Q);
    path.push_back(Q->cspace->goal_);
    robot->best_any_angle_path = path;
    cout << "Path: " << endl;
    double path_a, path_b, path_c;
    vector<double> avg_thetas;
    Eigen::VectorXd current_point, prev_point;
    for( int i = 0; i < path.size(); i++ ) {
        cout << path.at(i) << endl << endl;
        if(i==0) continue;
        current_point = path.at(i);
        prev_point = path.at(i-1);
        path_a = (current_point(1)-prev_point(1))
                /(current_point(0)-prev_point(0));
        path_b = -1;
        path_c = min(abs(current_point(1)),abs(prev_point(1)));
        lines.insert(lines.begin(), Eigen::Vector3d(path_a,path_b,path_c));
        angle = atan2(prev_point(1)-current_point(1),
                      prev_point(0)-current_point(0));
        x += cos(angle);
        y += sin(angle);
        avg_thetas.push_back(atan2(y,x));
    }

    vis = make_shared<thread>(visualizer, kd_tree, robot, Q);

    /// End Initialization

    /// Main loop
    cout << "RRTx Main Loop" << endl;
    startTime = chrono::high_resolution_clock::now();
    double slice_counter = 0;
    double slice_start
     = chrono::duration_cast<chrono::nanoseconds>(startTime-startTime).count();
    Q->cspace->start_time_ns_ = slice_start;
    Q->cspace->time_elapsed_ = 0.0;
    double slice_end;

    double now_time = getTimeNs(startTime);
    double trunc_elapsed_time;
    double time_start, time_end, iter_start, iter_end;

    double old_rrtLMC, current_distance, move_distance, this_dist;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;
    shared_ptr<ListNode> list_item;
    bool removed, added;
    shared_ptr<Obstacle> obstacle;

    {
        lock_guard<mutex> lock(robot->robot_mutex);
        prev_pose = robot->robot_pose;
    }

    // For importance sampling
    double f_uniform = 0.9;     // proportion to importance sample
    double position_bias;       // cartesian bias
    double theta_bias = PI/10;  // orientation bias

    current_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                root->position);

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->cspace->saturation_delta_, p.ball_constant*(
                                pow(log(1+kd_tree->treeSize)
                                    /(kd_tree->treeSize),
                                    1/Q->cspace->num_dimensions_) ));
        now_time = getTimeNs(startTime);

        slice_end = (1+slice_counter)*p.slice_time;

        /// Check for warm up time
        if(Q->cspace->in_warmup_time_
                && Q->cspace->warmup_time_ < Q->cspace->time_elapsed_) {
            Q->cspace->in_warmup_time_ = false;
        }


        /// Update Obstacles
        //Obstacle::UpdateObstacles(Q->cspace);


        // Remove obstacles
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            list_item = Q->cspace->obstacles_->front_;
        }
        removed = false;
        while(list_item != list_item->child_) {
            obstacle = list_item->obstacle_;

            if(!obstacle->sensible_obstacle_ && obstacle->obstacle_used_
                    && (obstacle->start_time_ + obstacle->life_span_
                        <= Q->cspace->time_elapsed_)) {
                // Time to remove obstacle
                cout << "time to remove" << endl;
                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
                removed = true;
            } else if(obstacle->sensible_obstacle_
                      && !obstacle->obstacle_used_after_sense_
                      && (Q->cspace->distanceFunction(prev_pose,
                                                 obstacle->position_)
                          < robot_sensor_range + obstacle->radius_)) {
                // Place to remove obstacle
                // The space that used to be in this obstacle was never
                // sampled so there will be a hole in the graph where it used
                // to be.
                // So require that the next few samples come from that space
                cout << "place to remove" << endl;
                RandomSampleObs(Q->cspace,kd_tree,obstacle);
                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
                obstacle->sensible_obstacle_ = false;
                obstacle->start_time_ = INF;
                removed = true;

            } else if(Q->cspace->space_has_time_
                      && (obstacle->next_direction_change_time_ > prev_pose(2))
                      && obstacle->last_direction_change_time_ != prev_pose(2)) {
                cout << "direction change" << endl;
                // A moving obstacle with unknown path is changing direction,
                // so remove its old anticipated trajectory
                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
                obstacle->obstacle_used_ = true;
                removed = true;
            }
            list_item = list_item->child_;
        }
        if(removed) {
            cout << "Obstacle Removed" << endl;
            reduceInconsistency(Q,Q->cspace->move_goal_,Q->cspace->robot_radius_,
                                root,hyper_ball_rad);
        }

        // Add Obstacles
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            list_item = Q->cspace->obstacles_->front_;
        }
        added = false;
        Eigen::VectorXd robot_pose;
        {
            lock_guard<mutex> lock(robot->robot_mutex);
            robot_pose = robot->robot_pose;
        }
        while(list_item != list_item->child_) {
            obstacle = list_item->obstacle_;

            if(!obstacle->sensible_obstacle_ && !obstacle->obstacle_used_
                    && obstacle->start_time_ <= Q->cspace->time_elapsed_
                    && Q->cspace->time_elapsed_
                    <= obstacle->start_time_ + obstacle->life_span_) {
                // Time to add
                cout << "time to add" << endl;
                /// Adding this causes program to slow considerably
                obstacle->obstacle_used_ = true;
                AddNewObstacle(kd_tree,Q,obstacle,root);
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
                added = true;
            } else if(obstacle->sensible_obstacle_
                      && obstacle->obstacle_used_after_sense_
                      && (Q->cspace->distanceFunction(robot_pose,
                                                 obstacle->position_))
                      < robot_sensor_range + obstacle->radius_) {
                // Place to add
                cout << "place to add" << endl;
                obstacle->obstacle_used_ = true;
                AddNewObstacle(kd_tree,Q,obstacle,root);
                obstacle->sensible_obstacle_ = false;
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
                added = true;
            } else if(Q->cspace->space_has_time_
                      && obstacle->next_direction_change_time_ > robot_pose(3)
                      && obstacle->last_direction_change_time_
                      != robot_pose(3)) {
                // Time that a moving obstacle with unknown path changes
                // direction
                cout << "direction change" << endl;
                obstacle->obstacle_used_ = true;
                obstacle->ChangeObstacleDirection(Q->cspace,robot_pose(3));
                AddNewObstacle(kd_tree,Q,obstacle,root);
                obstacle->last_direction_change_time_ = robot_pose(3);
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
            } else if(Q->cspace->warmup_time_just_ended_
                      && obstacle->obstacle_used_) {
                // Warm up time is over, so we need to treat all obstacles
                // as if they have just been added
//                cout << "finished warm up time" << endl;
//                AddNewObstacle(kd_tree,Q,obstacle,root);
                // Now check the robot's current move to its target
//                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
//                    robot->current_move_invalid = true;
//                added = true;
            }
            list_item = list_item->child_;
        }
        if(added) {
            propogateDescendants(Q,kd_tree,robot);
            if(!markedOS(Q->cspace->move_goal_)) verifyInQueue(Q,Q->cspace->move_goal_);
            cout << "Obstacle Added" << endl;
            reduceInconsistency(Q,Q->cspace->move_goal_,Q->cspace->robot_radius_,
                                root,hyper_ball_rad);
        }


        Q->cspace->time_elapsed_ = (getTimeNs(startTime)
                             - Q->cspace->start_time_ns_)/1000000000.0;
        if(Q->cspace->time_elapsed_ >= slice_end) {
            cout << "\nIteration: " << i++ << endl << "---------" << endl;
            iter_start = getTimeNs(startTime);

            slice_start = now_time;
            slice_end = (++slice_counter)*p.slice_time;
            trunc_elapsed_time = floor(Q->cspace->time_elapsed_*1000.0)/1000.0;

            /// Move robot
            if(Q->cspace->time_elapsed_ > p.planning_only_time + p.slice_time) {
                if(p.move_robot_flag) {
                    time_start = getTimeNs(startTime);
                    MoveRobot(Q,kd_tree,root,
                              p.slice_time,hyper_ball_rad,robot);
                    time_end = getTimeNs(startTime);
                    if(timingex) cout << "MoveRobot: "
                                      << (time_end - time_start)/MICROSECOND
                                      << endl;
//                    // Record data (robot path)
//                    rHist.row(histPos++) = robot->robot_pose;
//                    if( robot->robot_edge != prev_edge) {
//                        // Record data (edges)
//                        kdEdge.row(kdEdgePos++)
//                                = robot->robot_edge->start_node_->position;
//                        kdEdge.row(kdEdgePos++)
//                                = robot->robot_edge->end_node_->position;
//                    }
                } else { cout << "robot not moved" << endl; break; }
            }
            prev_edge = robot->robot_edge;


            /// Make graph consistent
            time_start = getTimeNs(startTime);
            reduceInconsistency(Q,Q->cspace->move_goal_, Q->cspace->robot_radius_,
                                root, hyper_ball_rad);
            time_end = getTimeNs(startTime);
            if(timingex) cout << "reduceInconsistency: "
                              << (time_end - time_start)/MICROSECOND << " ms" << endl;
            if(Q->cspace->move_goal_->rrtLMC != old_rrtLMC) {
                old_rrtLMC = Q->cspace->move_goal_->rrtLMC;
            }

            /// Check for completion
            current_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                        root->position);
            move_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                     prev_pose);
            cout << "Distance to goal: " << current_distance << endl;
            if(current_distance < p.goal_threshold) {
                cout << "Reached goal" << endl;
                break;
            } else if( move_distance > 10) {
                cout << "Impossible move" <<endl;
                break;
            }
            prev_pose = robot->robot_pose;


            /// Sample free space
            shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
            shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
            shared_ptr<double> closest_dist = make_shared<double>(INF);

            bool importance_sampling = true;
            while(importance_sampling) {
                new_node = randNodeOrFromStack(Q->cspace);

                if(new_node->kdInTree) continue;

                kd_tree->kdFindNearest(closest_node,closest_dist,
                                      new_node->position);

                /// Saturate new node
                this_dist = kd_tree->distanceFunction(new_node->position,
                                                        closest_node->position);
                if(this_dist > Q->cspace->saturation_delta_ && new_node != Q->cspace->goal_node_) {
                    time_start = getTimeNs(startTime);
                    Edge::Saturate(new_node->position, closest_node->position,
                                   Q->cspace->saturation_delta_, this_dist);
                    time_end = getTimeNs(startTime);
                    if(timingex) cout << "Saturate: "
                                      << (time_end - time_start)/MICROSECOND
                                      << endl;
                }

                /// Check for obstacles
                //time_start = getTimeNs(startTime);
                if(ExplicitNodeCheck(Q,new_node)) continue;
                //time_end = getTimeNs(startTime);
                if(timingex) cout << "ExplicitNodeCheck: "
                                  << (time_end - time_start)/MICROSECOND << " ms" << endl;

                /// Do importance sampling f_uniform*100% of the time
                if(randDouble(0,1) > f_uniform) break;

                if(kd_tree->distanceFunction(new_node->position,
                                             kd_tree->root->position)
                        > kd_tree->distanceFunction(Q->cspace->goal_node_->position,
                                                    kd_tree->root->position))
                    continue;

                if(kd_tree->distanceFunction(new_node-> position,
                                             Q->cspace->goal_node_->position)
                        > kd_tree->distanceFunction(Q->cspace->goal_node_->position,
                                                    kd_tree->root->position))
                    continue;

                // Find the current node closest line
                double dist_node_to_path;
                double min_dist_node_to_path = INF;
                int min_dist_node_to_path_index = 0;
                for(int i = 1; i < robot->best_any_angle_path.size(); i++) {
                    dist_node_to_path = DistanceSqrdPointToSegment(new_node->position,
                                             robot->best_any_angle_path.at(i-1).head(2),
                                             robot->best_any_angle_path.at(i).head(2));
                    if(dist_node_to_path < min_dist_node_to_path) {
                        min_dist_node_to_path = dist_node_to_path;
                        min_dist_node_to_path_index = i-1;
                    }
                }

                /// Theta bias
                new_node->position(2) = randDouble(avg_thetas[min_dist_node_to_path_index]-theta_bias,
                                                   avg_thetas[min_dist_node_to_path_index]+theta_bias);

                /// Position bias
                position_bias = hyper_ball_rad;
                double dist = DistanceSqrdPointToSegment(new_node->position,
                            path.at(min_dist_node_to_path_index).head(2),
                            path.at(min_dist_node_to_path_index+1).head(2));
                if(dist > position_bias) continue;

                importance_sampling = false;
            }

            /// Extend graph
            if(Extend(kd_tree,Q,new_node,closest_node,
                      Q->cspace->saturation_delta_,hyper_ball_rad,Q->cspace->move_goal_)) {
//                // Record data (kd-tree)
//                kdTree.row(kdTreePos++) = new_node->position;
            }

            /// Make graph consistent
            reduceInconsistency(Q,Q->cspace->move_goal_,Q->cspace->robot_radius_,
                                root,hyper_ball_rad);
            if(Q->cspace->move_goal_->rrtLMC != old_rrtLMC) {
                old_rrtLMC = Q->cspace->move_goal_->rrtLMC;
            }

            // If the difference between the first node and the root is
            // > delta, then set its LMC to INF so it can be recalculated
            // when the next node is added
            /// In general this shouldn't happen because of Saturate()
            if(i == 1 && new_node->rrtParentUsed && (new_node->rrtLMC
                   - new_node->rrtParentEdge->end_node_->rrtLMC > Q->cspace->saturation_delta_)) {
                new_node->rrtLMC = INF;
            }

            iter_end = getTimeNs(startTime);
            cout << "Iteration: "
                 << (iter_end - iter_start)/MICROSECOND << " ms" << endl;
        }
    }
    PrintRrtxPath(Q->cspace->goal_node_);
    return robot;
}

/// Distance Function for use in the RRTx algorithm
// This was created by Michael Otte (R3SDist) using [x,y,t,theta] (Dubin's)
double distance_function( Eigen::VectorXd a, Eigen::VectorXd b )
{
    Eigen::ArrayXd temp = a.head(2) - b.head(2);
    temp = temp*temp;
    return sqrt( temp.sum()
                 + pow( std::min( std::abs(a(2)-b(2)),
                                  std::min(a(2),b(2)) + 2.0*PI
                                    - std::max(a(2),b(2)) ), 2 ) );
}

int main( int argc, char* argv[] )
{
    /// C-Space
    int dims = 3;                   // x,y,theta
    double envRad = 10.0;           // dimensions of the c-spcae
    Eigen::Vector3d lbound, ubound;
    lbound << -envRad, -envRad, 0.0;
    ubound << envRad, envRad, 2*PI;

    Eigen::Vector3d start, goal;
    start << 0.0, 0.0, -3*PI/4;
    goal << 20.0, 20.0, -3*PI/4;    // where the robot begins

    shared_ptr<ConfigSpace> cspace
            = make_shared<ConfigSpace>(dims,lbound,ubound,start,goal);
    cspace->setDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1;
    cspace->robot_radius_ = 0.5;
    cspace->robot_velocity_ = 20.0;
    cspace->min_turn_radius_ = 1.0;
    cspace->prob_goal_ = 0.01;           // probability of sampling the goal node
    cspace->space_has_time_ = false;
    cspace->space_has_theta_ = true;   // Dubin's car model

    /// K-D Tree
    // Dubin's model wraps theta (4th entry) at 2pi
    Eigen::VectorXi wrap_vec(1);    // 1 wrapping dimension
    wrap_vec(0) = 2;
    Eigen::VectorXd wrap_points_vec(1);
    wrap_points_vec(0) = 2.0*PI;

    /// Parameters
    string alg_name = "RRTx";       // running RRTx
    string obstacle_file = argv[1]; // obstacle file
    double plan_time = 50.0;        // plan *only* for this long
    double slice_time = 1.0/100;    // iteration time limit
    double delta = 5.0;             // distance between graph nodes
    double ball_const = 100.0;      // search d-ball radius
    double change_thresh = 1.0;     // node change detection
    double goal_thresh = 0.5;       // goal detection
    bool move_robot = true;         // move robot after plan_time/slice_time

    /// Read in Obstacles
    Obstacle::ReadObstaclesFromFile(obstacle_file, cspace);

    // Create a new problem for RRTx
    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wrap_vec, wrap_points_vec);

    // Pointer to visualizer thread (created in RRTX())
    shared_ptr<thread> vis_thread;

    /// Run RRTx
    shared_ptr<RobotData> robot_data = Rrtx(problem, vis_thread);

    /// Save data
    // Calculate and display distance traveled
    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = robot_data->robot_move_path.block(0,0,
                                      robot_data->num_robot_move_points-1,3);
    lastpoints = robot_data->robot_move_path.block(1,0,
                                     robot_data->num_robot_move_points-1,3);
    diff = firstpoints - lastpoints;
    diff = diff*diff;
    for( int i = 0; i < diff.rows(); i++ ) {
        diff.col(0)(i) = diff.row(i).sum();
    }

    double moveLength = diff.col(0).sqrt().sum();
    cout << "Robot traveled: " << moveLength
              << " units" << endl;

    // Calculate and display time elapsed
    double totalTime = getTimeNs(startTime);
    cout << "Total time: " << totalTime/1000000000.0
              << " s" << endl;

//    ofstream ofs;
//    // Export robot path to file
//    ofs.open( "robotPath.txt", ofstream::out );
//    for( int j = 0; j < rHist.rows(); j++ ) {
//        ofs << rHist.row(j);
//        ofs << "\n";
//    }
//    ofs.close();

//    // Export kd-tree to file
//    ofs.open("kdTree.txt", ofstream::out );
//    for( int k = 0; k < kdTree.rows(); k++ ) {
//        ofs << kdTree.row(k);
//        ofs << "\n";
//    }
//    ofs.close();

//    ofs.open("kdEdge.txt", ofstream::out);
//    for(int p = 0; p < kdEdge.rows(); p += 2) {
//        ofs << kdEdge.row(p) << "\n" << kdEdge.row(p+1) << "\n";
//    }
//    ofs.close();

//    cout <<"Data written to kdTree.txt, kdEdge.txt, and robotPath.txt"<< endl;
    vis_thread->join();

    return 0;
}
