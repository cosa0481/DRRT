/* largetest.cpp
 * Corin Sandford
 * Fall 2016
 * This file is made into an executable: 'test'
 * Used to test high level things in the library
 * To define distance function just implement :
 *      'double distanceFunction(Eigen::VectorXd x, Eigen::VectorXd y)'
 * above the main function
 */

#include <DRRT/drrt.h>
#include <DRRT/visualizer.h>

using namespace std;

// Variables for saving data
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
        leaf = leaf->rrtParentEdge->end_node_;
    }
    cout << leaf->position << endl;
}


/// AlGORITHM CONTROL FUNCTION
// This function runs RRTx with the parameters defined in main()
shared_ptr<RobotData> RRTX(Problem p, shared_ptr<thread> &vis)
{
    // Used for "sensing" obstacles (should make input, from DRRT.jl)
    //double robotSensorRange = 20.0;

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

    /// KD-Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(Q->cspace->num_dimensions_,p.wraps,p.wrap_points);
    kd_tree->setDistanceFunction(Q->cspace->distanceFunction);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->cspace->start_);
    //explicitNodeCheck(S,root);
    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;
    root->rrtParentEdge = Edge::NewEdge(Q->cspace,kd_tree,root,root);
    root->rrtParentUsed = false;
    kd_tree->kdInsert(root);
    kdTree.row(kdTreePos++) = root->position;

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->cspace->goal_);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
    kdTree.row(kdTreePos++) = goal->position;

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

    shared_ptr<thread> vis_thread
            = make_shared<thread>(visualizer,kd_tree,robot,Q);
    vis = vis_thread;

    /// End Initialization

    /// Main loop
    startTime = chrono::high_resolution_clock::now();
    double slice_counter = 0;
    double slice_start
     = chrono::duration_cast<chrono::nanoseconds>(startTime-startTime).count();
    Q->cspace->start_time_ns_ = slice_start;
    Q->cspace->time_elapsed_ = 0.0;
    double slice_end;

    double now_time = getTimeNs(startTime);
    double trunc_elapsed_time;

    double old_rrtLMC, current_distance, move_distance, this_dist;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;

    current_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                root->position);
    prev_pose = robot->robot_pose;

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->cspace->saturation_delta_, p.ball_constant*(
                                pow(log(1+kd_tree->treeSize)/(kd_tree->treeSize),
                                    1/Q->cspace->num_dimensions_) ));
        now_time = getTimeNs(startTime);

        slice_end = (1+slice_counter)*p.slice_time;

        /// Check for warm up time
        if(Q->cspace->in_warmup_time_
                && Q->cspace->warmup_time_ < Q->cspace->time_elapsed_) {
            Q->cspace->in_warmup_time_ = false;
        }

        Q->cspace->time_elapsed_ = (getTimeNs(startTime)
                             - Q->cspace->start_time_ns_)/1000000000.0;
        if(Q->cspace->time_elapsed_ >= slice_end) {
            cout << "\nIteration: " << i++ << endl << "---------" << endl;

            slice_start = now_time;
            slice_end = (++slice_counter)*p.slice_time;
            trunc_elapsed_time = floor(Q->cspace->time_elapsed_*1000.0)/1000.0;

            /// Move robot
            if(Q->cspace->time_elapsed_ > p.planning_only_time + p.slice_time) {
                if(p.move_robot_flag) {
                    MoveRobot(Q,kd_tree,root,
                              p.slice_time,hyper_ball_rad,robot);
                    // Record data (robot path)
                    rHist.row(histPos++) = robot->robot_pose;
                    if( robot->robot_edge != prev_edge) {
                        // Record data (edges)
                        kdEdge.row(kdEdgePos++)
                                = robot->robot_edge->start_node_->position;
                        kdEdge.row(kdEdgePos++)
                                = robot->robot_edge->end_node_->position;
                    }
                } else { cout << "robot not moved" << endl; break; }
            }
            prev_edge = robot->robot_edge;


            /// Make graph consistent
            reduceInconsistency(Q,Q->cspace->move_goal_, Q->cspace->robot_radius_,
                                root, hyper_ball_rad);
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

            if( i == 1 ) { printRRTxPath(robot->next_move_target); }
            if( !robot->moving ) {
                /// Sample free space
                shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
                shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
                shared_ptr<double> closest_dist = make_shared<double>(INF);

                new_node = randNodeOrFromStack(Q->cspace);
                if(new_node->kdInTree) continue;

                kd_tree->kdFindNearest(closest_node,closest_dist,
                                      new_node->position);

                /// Saturate new node
                this_dist = kd_tree->distanceFunction(new_node->position,
                                                        closest_node->position);
                if(this_dist > Q->cspace->saturation_delta_ && new_node != Q->cspace->goal_node_) {
                    Edge::Saturate(new_node->position, closest_node->position,
                                   Q->cspace->saturation_delta_, this_dist);
                }

                /// Check for obstacles
                //bool explicitly_unsafe;
                //explicitNodeCheck(explicitly_unsafe,Q->cspace,new_node)
                //if(explicitly_unsafe) continue;

                /// Extend graph
                if(Extend(kd_tree,Q,new_node,closest_node,
                          Q->cspace->saturation_delta_,hyper_ball_rad,Q->cspace->move_goal_)) {
                    // Record data (kd-tree)
                    kdTree.row(kdTreePos++) = new_node->position;
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
            }
        }
    }
    printRRTxPath(Q->cspace->goal_node_);
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

int main()
{
    /// C-Space
    int dims = 3;                   // x,y,theta
    double envRad = 50.0;           // dimensions of the c-spcae
    Eigen::Vector3d lbound, ubound;
    lbound << -envRad, -envRad, 0.0;
    ubound << envRad, envRad, 2*PI;

    Eigen::Vector3d start, goal;
    start << 0.0, 0.0, -3*PI/4;
    goal << 150.0, 150.0, -3*PI/4;    // where the robot begins

    shared_ptr<ConfigSpace> cspace
            = make_shared<ConfigSpace>(dims,lbound,ubound,start,goal);
    cspace->setDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1;   // distance for collision
    cspace->robot_radius_ = 0.5;
    cspace->robot_velocity_ = 10.0;
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
    double plan_time = 300.0;       // plan *only* for this long
    double slice_time = 1.0/100;    // iteration time limit
    double delta = 10.0;            // distance between graph nodes
    double ball_const = 100.0;      // search d-ball radius
    double change_thresh = 1.0;     // node change detection
    double goal_thresh = 0.5;       // goal detection
    bool move_robot = true;         // move robot after plan_time/slice_time

    // Create a new problem for RRTx
    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wrap_vec, wrap_points_vec);

    // Visualizer thread to join
    shared_ptr<thread> visualizer_thread;

    /// Run RRTx
    shared_ptr<RobotData> robot_data = RRTX(problem,visualizer_thread);

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
    visualizer_thread->join();

    return 0;
}
