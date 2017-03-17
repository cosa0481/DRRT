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

// Variables for saving data from Dubin's space
int histPos = 0,
    kdTreePos = 0,
    kdEdgePos = 0;

Eigen::MatrixXd rHist(MAXPATHNODES,3),
                kdTree(MAXPATHNODES,3),
                kdEdge(MAXPATHNODES,3);

chrono::time_point<chrono::high_resolution_clock> startTime;

void printRRTxPath(shared_ptr<KDTreeNode> &leaf)
{
    std::cout << "\nRRTx Path" << std::endl;
    while(leaf->rrtParentUsed) {
        cout << "pose: " << leaf->rrtLMC << "\n" << leaf->position << endl;
        cout << "VVVVVVVV" << endl;
        leaf = leaf->rrtParentEdge->endNode;
    }
    cout << leaf->position << endl;
}

/// AlGORITHM CONTROL FUNCTION
// This function runs RRTx with the parameters defined in main()
shared_ptr<RobotData> RRTX(Problem p, shared_ptr<thread> &vis)
{
    // Used for "sensing" obstacles (should make input, from DRRT.jl)
    double robot_sensor_range = 20.0;

    /// Initialize

    /// Queue
    shared_ptr<Queue> Q = make_shared<Queue>();
    Q->Q = make_shared<BinaryHeap>(false); // priority queue use Q functions
    Q->OS = make_shared<JList>(true); // obstacle stack uses KDTreeNodes
    Q->changeThresh = p.change_threshold;
    Q->type = p.search_type;
    Q->S = p.c_space;
    Q->S->sampleStack = make_shared<JList>(true); // uses KDTreeNodes
    Q->S->delta = p.delta;

    /// K-D Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(Q->S->d,p.wraps,p.wrap_points);
    kd_tree->setDistanceFunction(Q->S->distanceFunction);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->S->start);
    ExplicitNodeCheck(Q,root);
    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;
    root->rrtParentEdge = Edge::newEdge(Q->S,kd_tree,root,root);
    root->rrtParentUsed = false;
    kd_tree->kdInsert(root);
    kdTree.row(kdTreePos++) = root->position;

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->S->goal);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
    kdTree.row(kdTreePos++) = goal->position;

    Q->S->goalNode = goal;
    Q->S->root = root;
    Q->S->moveGoal = goal;
    Q->S->moveGoal->isMoveGoal = true;

    /// Robot
    shared_ptr<RobotData> robot
            = make_shared<RobotData>(Q->S->goal, goal, MAXPATHNODES, Q->S->d);

    if(Q->S->spaceHasTime) {
        addOtherTimesToRoot(Q->S,kd_tree,goal,root,Q->type);
    }

    shared_ptr<ListNode> os;
    {
        lock_guard<mutex> lock(Q->S->cspace_mutex_);
        os = Q->S->obstacles->front_;
    }
    os->obstacle_->obstacle_used_ = true;
    AddNewObstacle(kd_tree,Q,os->obstacle_,root,robot);

    /// Save the path to vector of anyangle path lines
    /// Path in form b*y = a*x + c
    vector<Eigen::Vector3d> lines;
    double angle, x = 0, y = 0;
    vector<Eigen::VectorXd> path = theta_star(Q->S);
    path.push_back(Q->S->goal);
    robot->best_any_angle_path = path;
    cout << "Path: " << endl;
    double path_a, path_b, path_c;
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
        lines.push_back(Eigen::Vector3d(path_a,path_b,path_c));
        angle = atan2(prev_point(1)-current_point(1),
                      prev_point(0)-current_point(0));
        x += cos(angle);
        y += sin(angle);
    }
    double avg_theta = atan2(y,x); // Average heading of path

    vis = make_shared<thread>(visualizer, kd_tree, robot, Q);

    /// End Initialization

    /// Main loop
    cout << "RRTx Main Loop" << endl;
    startTime = chrono::high_resolution_clock::now();
    double slice_counter = 0;
    double slice_start
     = chrono::duration_cast<chrono::nanoseconds>(startTime-startTime).count();
    Q->S->startTimeNs = slice_start;
    Q->S->timeElapsed = 0.0;
    double slice_end;

    double now_time = getTimeNs(startTime);
    double trunc_elapsed_time;
    double time_start, time_end, iter_start, iter_end;

    double old_rrtLMC, current_distance, move_distance, this_dist;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;
    shared_ptr<ListNode> list_item;
    bool /*removed,*/ added;
    shared_ptr<Obstacle> obstacle;

    {
        lock_guard<mutex> lock(robot->robotMutex_);
        prev_pose = robot->robotPose;
    }

    // For importance sampling
    double f_uniform = 0.7;     // proportion to sample X_free
    double position_bias;       // cartesian bias
    double theta_bias = PI/10;  // orientation bias

    current_distance = kd_tree->distanceFunction(robot->robotPose,
                                                root->position);

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->S->delta, p.ball_constant*(
                                pow(log(1+kd_tree->treeSize)
                                    /(kd_tree->treeSize),
                                    1/Q->S->d) ));
        now_time = getTimeNs(startTime);

        slice_end = (1+slice_counter)*p.slice_time;

        /// Check for warm up time
        if(Q->S->inWarmupTime
                && Q->S->warmupTime < Q->S->timeElapsed) {
            Q->S->inWarmupTime = false;
        }


        /// Update Obstacles
        //Obstacle::UpdateObstacles(Q->S);


//        // Remove obstacles
//        {
//            lock_guard<mutex> lock(Q->S->cspace_mutex_);
//            list_item = Q->S->obstacles->front_;
//        }
//        removed = false;
//        while(list_item != list_item->child_) {
//            obstacle = list_item->obstacle_;

//            if(!obstacle->sensible_obstacle_ && obstacle->obstacle_used_
//                    && (obstacle->start_time_ + obstacle->life_span_
//                        <= Q->S->timeElapsed)) {
//                // Time to remove obstacle
//                cout << "time to remove" << endl;
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->S->timeElapsed,Q->S->moveGoal);
//                removed = true;
//            } else if(obstacle->sensible_obstacle_
//                      && !obstacle->obstacle_used_after_sense_
//                      && (Q->S->distanceFunction(prev_pose,
//                                                 obstacle->position_)
//                          < robot_sensor_range + obstacle->radius_)) {
//                // Place to remove obstacle
//                // The space that used to be in this obstacle was never
//                // sampled so there will be a hole in the graph where it used
//                // to be.
//                // So require that the next few samples come from that space
//                cout << "place to remove" << endl;
//                RandomSampleObs(Q->S,kd_tree,obstacle);
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->S->timeElapsed,Q->S->moveGoal);
//                obstacle->sensible_obstacle_ = false;
//                obstacle->start_time_ = INF;
//                removed = true;

//            } else if(Q->S->spaceHasTime
//                      && (obstacle->next_direction_change_time_ > prev_pose(2))
//                      && obstacle->last_direction_change_time_ != prev_pose(2)) {
//                cout << "direction change" << endl;
//                // A moving obstacle with unknown path is changing direction,
//                // so remove its old anticipated trajectory
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->S->timeElapsed,Q->S->moveGoal);
//                obstacle->obstacle_used_ = true;
//                removed = true;
//            }
//            list_item = list_item->child_;
//        }
//        if(removed) {
//            cout << "Obstacle Removed" << endl;
//            reduceInconsistency(Q,Q->S->moveGoal,Q->S->robotRadius,
//                                root,hyper_ball_rad);
//        }

        // Add Obstacles
        {
            lock_guard<mutex> lock(Q->S->cspace_mutex_);
            list_item = Q->S->obstacles->front_;
        }
        added = false;
        Eigen::VectorXd robot_pose;
        {
            lock_guard<mutex> lock(robot->robotMutex_);
            robot_pose = robot->robotPose;
        }
        while(list_item != list_item->child_) {
            obstacle = list_item->obstacle_;

            if(!obstacle->sensible_obstacle_ && !obstacle->obstacle_used_
                    && obstacle->start_time_ <= Q->S->timeElapsed
                    && Q->S->timeElapsed
                    <= obstacle->start_time_ + obstacle->life_span_) {
                // Time to add
                cout << "time to add" << endl;
                /// Adding this causes program to slow considerably
                obstacle->obstacle_used_ = true;
                AddNewObstacle(kd_tree,Q,obstacle,root,robot);
                added = true;
            } else if(obstacle->sensible_obstacle_
                      && obstacle->obstacle_used_after_sense_
                      && (Q->S->distanceFunction(robot_pose,
                                                 obstacle->position_))
                      < robot_sensor_range + obstacle->radius_) {
                // Place to add
                cout << "place to add" << endl;
                obstacle->obstacle_used_ = true;
                AddNewObstacle(kd_tree,Q,obstacle,root,robot);
                obstacle->sensible_obstacle_ = false;
                added = true;
            } else if(Q->S->spaceHasTime
                      && obstacle->next_direction_change_time_ > robot_pose(3)
                      && obstacle->last_direction_change_time_
                      != robot_pose(3)) {
                // Time that a moving obstacle with unknown path changes
                // direction
                cout << "direction change" << endl;
                obstacle->obstacle_used_ = true;
                obstacle->ChangeObstacleDirection(Q->S,robot_pose(3));
                AddNewObstacle(kd_tree,Q,obstacle,root,robot);
                obstacle->last_direction_change_time_ = robot_pose(3);
            } else if(Q->S->warmup_time_just_ended
                      && obstacle->obstacle_used_) {
                // Warm up time is over, so we need to treat all obstacles
                // as if they have just been added
//                cout << "finished warm up time" << endl;
//                AddNewObstacle(kd_tree,Q,obstacle,root,robot);
//                added = true;
            }
            list_item = list_item->child_;
        }
        if(added) {
            propogateDescendants(Q,kd_tree,robot);
            if(!markedOS(Q->S->moveGoal)) verifyInQueue(Q,Q->S->moveGoal);
            cout << "Obstacle Added" << endl;
            reduceInconsistency(Q,Q->S->moveGoal,Q->S->robotRadius,
                                root,hyper_ball_rad);
        }


        Q->S->timeElapsed = (getTimeNs(startTime)
                             - Q->S->startTimeNs)/1000000000.0;
        if(Q->S->timeElapsed >= slice_end) {
            cout << "\nIteration: " << i++ << endl << "---------" << endl;
            iter_start = getTimeNs(startTime);

            slice_start = now_time;
            slice_end = (++slice_counter)*p.slice_time;
            trunc_elapsed_time = floor(Q->S->timeElapsed*1000.0)/1000.0;

            /// Move robot
            if(Q->S->timeElapsed > p.planning_only_time + p.slice_time) {
                if(p.move_robot_flag) {
                    time_start = getTimeNs(startTime);
                    MoveRobot(Q,kd_tree,root,
                              p.slice_time,hyper_ball_rad,robot);
                    time_end = getTimeNs(startTime);
                    if(timingex) cout << "MoveRobot: "
                                      << (time_end - time_start)/MICROSECOND
                                      << endl;
                    // Record data (robot path)
                    rHist.row(histPos++) = robot->robotPose;
                    if( robot->robotEdge != prev_edge) {
                        // Record data (edges)
                        kdEdge.row(kdEdgePos++)
                                = robot->robotEdge->startNode->position;
                        kdEdge.row(kdEdgePos++)
                                = robot->robotEdge->endNode->position;
                    }
                } else { cout << "robot not moved" << endl; break; }
            }
            prev_edge = robot->robotEdge;


            /// Make graph consistent
            time_start = getTimeNs(startTime);
            reduceInconsistency(Q,Q->S->moveGoal, Q->S->robotRadius,
                                root, hyper_ball_rad);
            time_end = getTimeNs(startTime);
            if(timingex) cout << "reduceInconsistency: "
                              << (time_end - time_start)/MICROSECOND << " ms" << endl;
            if(Q->S->moveGoal->rrtLMC != old_rrtLMC) {
                old_rrtLMC = Q->S->moveGoal->rrtLMC;
            }

            /// Check for completion
            current_distance = kd_tree->distanceFunction(robot->robotPose,
                                                        root->position);
            move_distance = kd_tree->distanceFunction(robot->robotPose,
                                                     prev_pose);
            cout << "Distance to goal: " << current_distance << endl;
            if(current_distance < p.goal_threshold) {
                cout << "Reached goal" << endl;
                break;
            } else if( move_distance > 10) {
                cout << "Impossible move" <<endl;
                break;
            }
            prev_pose = robot->robotPose;


            /// Sample free space
            shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
            shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
            shared_ptr<double> closest_dist = make_shared<double>(INF);

            bool importance_sampling = true;
            while(importance_sampling) {
                new_node = randNodeOrFromStack(Q->S);

                if(new_node->kdInTree) continue;

                kd_tree->kdFindNearest(closest_node,closest_dist,
                                      new_node->position);

                /// Saturate new node
                this_dist = kd_tree->distanceFunction(new_node->position,
                                                        closest_node->position);
                if(this_dist > Q->S->delta && new_node != Q->S->goalNode) {
                    time_start = getTimeNs(startTime);
                    Edge::saturate(new_node->position, closest_node->position,
                                   Q->S->delta, this_dist);
                    time_end = getTimeNs(startTime);
                    if(timingex) cout << "saturate: "
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
                        > kd_tree->distanceFunction(Q->S->goalNode->position,
                                                    kd_tree->root->position))
                    continue;

                if(kd_tree->distanceFunction(new_node-> position,
                                             Q->S->goalNode->position)
                        > kd_tree->distanceFunction(Q->S->goalNode->position,
                                                    kd_tree->root->position))
                    continue;

                /// Theta bias
                new_node->position(2) = randDouble(avg_theta-theta_bias,
                                                   avg_theta+theta_bias);

                /// Position bias
                double x = new_node->position(0);
                double y = new_node->position(1);
                double dist = INF;
                double min = INF;
                position_bias = hyper_ball_rad;
                for(int j = 0; j < lines.size(); j++) {
                    dist = abs(lines.at(j)(0)*x
                               + lines.at(j)(1)*y
                               + lines.at(j)(2))
                    / sqrt(pow(lines.at(j)(0),2)
                           + pow(lines.at(j)(1),2));
                    if(dist < min) min = dist;
                }
                if(min > position_bias) continue;

                importance_sampling = false;
            }

            /// Extend graph
            if(Extend(kd_tree,Q,new_node,closest_node,
                      Q->S->delta,hyper_ball_rad,Q->S->moveGoal)) {
                // Record data (kd-tree)
                kdTree.row(kdTreePos++) = new_node->position;
            }

            /// Make graph consistent
            reduceInconsistency(Q,Q->S->moveGoal,Q->S->robotRadius,
                                root,hyper_ball_rad);
            if(Q->S->moveGoal->rrtLMC != old_rrtLMC) {
                old_rrtLMC = Q->S->moveGoal->rrtLMC;
            }

            // If the difference between the first node and the root is
            // > delta, then set its LMC to INF so it can be recalculated
            // when the next node is added
            /// In general this shouldn't happen because of saturate()
            if(i == 1 && new_node->rrtParentUsed && (new_node->rrtLMC
                   - new_node->rrtParentEdge->endNode->rrtLMC > Q->S->delta)) {
                new_node->rrtLMC = INF;
            }

            iter_end = getTimeNs(startTime);
            cout << "Iteration: "
                 << (iter_end - iter_start)/MICROSECOND << " ms" << endl;
        }
    }
    printRRTxPath(Q->S->goalNode);
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

    shared_ptr<CSpace> cspace
            = make_shared<CSpace>(dims,lbound,ubound,start,goal);
    cspace->setDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1;
    cspace->robotRadius = 0.5;
    cspace->robotVelocity = 20.0;
    cspace->minTurningRadius = 1.0;
    cspace->pGoal = 0.01;           // probability of sampling the goal node
    cspace->spaceHasTime = false;
    cspace->spaceHasTheta = true;   // Dubin's car model

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
    shared_ptr<RobotData> robot_data = RRTX(problem, vis_thread);

    /// Save data
    // Calculate and display distance traveled
    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = robot_data->robotMovePath.block(0,0,
                                      robot_data->numRobotMovePoints-1,3);
    lastpoints = robot_data->robotMovePath.block(1,0,
                                     robot_data->numRobotMovePoints-1,3);
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

    ofstream ofs;
    // Export robot path to file
    ofs.open( "robotPath.txt", ofstream::out );
    for( int j = 0; j < rHist.rows(); j++ ) {
        ofs << rHist.row(j);
        ofs << "\n";
    }
    ofs.close();

    // Export kd-tree to file
    ofs.open("kdTree.txt", ofstream::out );
    for( int k = 0; k < kdTree.rows(); k++ ) {
        ofs << kdTree.row(k);
        ofs << "\n";
    }
    ofs.close();

    ofs.open("kdEdge.txt", ofstream::out);
    for(int p = 0; p < kdEdge.rows(); p += 2) {
        ofs << kdEdge.row(p) << "\n" << kdEdge.row(p+1) << "\n";
    }
    ofs.close();

    cout <<"Data written to kdTree.txt, kdEdge.txt, and robotPath.txt"<< endl;
    vis_thread->join();

    return 0;
}
