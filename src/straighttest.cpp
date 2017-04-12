/* straighttest.cpp
 * Corin Sandford
 * Spring 2017
 * Tests Dubin's model to make sure it
 * plans a straight line.
 */

#include <DRRT/drrt.h> // the RRTx library
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
    while(leaf->rrt_parent_used_) {
        cout << "pose: " << leaf->rrt_LMC_ << "\n" << leaf->position_ << endl;
        cout << "VVVVVVVV" << endl;
        leaf = leaf->rrt_parent_edge_->end_node_;
    }
    cout << leaf->position_ << endl;
}

/// Main control function
shared_ptr<RobotData> RRTX(Problem p, shared_ptr<thread> &vis)
{
    if(p.search_type != "RRTx") {
        std::cout << "y u no want RRTx?" << std::endl;
        exit(1);
    }

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

    /// KD-Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(Q->cspace->num_dimensions_,p.wraps_,p.wrap_points);
    kd_tree->SetDistanceFunction(Q->cspace->distanceFunction);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->cspace->start_);
    ExplicitNodeCheck(Q,root);
    root->rrt_tree_cost_ = 0.0;
    root->rrt_LMC_ = 0.0;
    root->rrt_parent_edge_ = Edge::NewEdge(Q->cspace,kd_tree,root,root);
    root->rrt_parent_used_ = false;
    kd_tree->KDInsert(root);
    kdTree.row(kdTreePos++) = root->position_;

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->cspace->goal_);
    goal->rrt_tree_cost_ = INF;
    goal->rrt_LMC_ = INF;
    kdTree.row(kdTreePos++) = goal->position_;

    Q->cspace->goal_node_ = goal;
    Q->cspace->root_ = root;
    Q->cspace->move_goal_ = goal;
    Q->cspace->move_goal_->is_move_goal_ = true;

    /// Robot
    shared_ptr<RobotData> robot
            = make_shared<RobotData>(Q->cspace->goal_, goal,
                                     MAXPATHNODES, Q->cspace->num_dimensions_);

    if(Q->cspace->space_has_time_) {
        addOtherTimesToRoot(Q->cspace,kd_tree,goal,root,Q->type);
    }

    shared_ptr<thread> visualizer_thread
            = make_shared<thread>(visualizer,kd_tree,robot,Q);
    vis = visualizer_thread;

    /// End Initialization

    /// Main loop
    startTime = chrono::high_resolution_clock::now();
    double slice_counter = 0;
    double slice_start
     = chrono::duration_cast<chrono::nanoseconds>(startTime-startTime).count();
    Q->cspace->start_time_ns_ = slice_start;
    Q->cspace->time_elapsed_ = 0.0;
    double slice_end;

    double now_time = GetTimeNs(startTime);
    double trunc_elapsed_time;

    double current_distance;
    double move_distance;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;
    shared_ptr<ListNode> list_item;
    bool /*removed,*/ added;
    shared_ptr<Obstacle> obstacle;

    {
        cout << "locking" << endl;
        lock_guard<mutex> lock(robot->robot_mutex);
        prev_pose = robot->robot_pose;
        cout << "unlocking" << endl;
    }

    current_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                root->position_);

    shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
    shared_ptr<double> closest_dist = make_shared<double>(INF);

///////////////////
    Eigen::Vector3d position_;

    /// Test obstacle avoidance
    position_(0) = 2.5;
    position_(1) = 2.5;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node1 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node1->position_);
    Extend(kd_tree,Q,node1,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node1->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 5;
    position_(1) = 5;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node2 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node2->position_);
    Extend(kd_tree,Q,node2,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node2->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 10;
    position_(1) = 10;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node3 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node3->position_);
    Extend(kd_tree,Q,node3,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node3->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 15;
    position_(1) = 15;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node4 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node4->position_);
    Extend(kd_tree,Q,node4,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node4->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 8;
    position_(1) = 15;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node5 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node5->position_);
    Extend(kd_tree,Q,node5,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node5->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 5;
    position_(1) = 11;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node6 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node6->position_);
    Extend(kd_tree,Q,node6,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node6->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    position_(0) = 20;
    position_(1) = 20;
    position_(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node7 = make_shared<KDTreeNode>(position_);
    kd_tree->KDFindNearest(closest_node,closest_dist,node7->position_);
    Extend(kd_tree,Q,node7,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
    kdTree.row(kdTreePos++) = node7->position_;
    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);


/// Test to plan path through uniform grid of points
//    position_(0) = 5;
//    position_(1) = 5;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node4 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node4->position_);
//    Extend(kd_tree,Q,node4,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node4->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    // If the difference between the first node and the root is
    // > delta, then set its LMC to INF so it can be recalculated
    // when the next node is added since if rrt_LMC_ - rrt_tree_cost_
    // > changeThreshold decides whether this is necessary.
    // This didn't work correctly because the code skips a node whose
    // parent is the root when it comes to rewiring In Neighbors.
    // Noticed this when adding 8,12 before 5,5 and 8,12 would
    // remain the rrtChild of the root instead of switching to 5,5
    /// In general this shouldn't happen because of saturation of nodes/* test.cpp
//    if( node4->rrt_LMC_ - node4->rrt_parent_edge_->end_node_->rrt_LMC_ > 10 )
//        node4->rrt_LMC_ = INF;

//    position_(0) = 8;
//    position_(1) = 12;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node1 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node1->position_);
//    Extend(kd_tree,Q,node1,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node1->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 15;
//    position_(1) = 15;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node5 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node5->position_);
//    Extend(kd_tree,Q,node5,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node5->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 7;
//    position_(1) = 3;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node7 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node7->position_);
//    Extend(kd_tree,Q,node7,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node7->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 10;
//    position_(1) = 10;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node3 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node3->position_);
//    Extend(kd_tree,Q,node3,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node3->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 18;
//    position_(1) = 22;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node10 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node10->position_);
//    Extend(kd_tree,Q,node10,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node10->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 22;
//    position_(1) = 18;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node11 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node11->position_);
//    Extend(kd_tree,Q,node11,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node11->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 13;
//    position_(1) = 17;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node8 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node8->position_);
//    Extend(kd_tree,Q,node8,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node8->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 20;
//    position_(1) = 20;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node12 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node12->position_);
//    Extend(kd_tree,Q,node12,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node12->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 3;
//    position_(1) = 7;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node6 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node6->position_);
//    Extend(kd_tree,Q,node6,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node6->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 12;
//    position_(1) = 8;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node2 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node2->position_);
//    Extend(kd_tree,Q,node2,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node2->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

//    position_(0) = 17;
//    position_(1) = 13;
//    position_(2) = -3*PI/4;
//    shared_ptr<KDTreeNode> node9 = make_shared<KDTreeNode>(position_);
//    kd_tree->KDFindNearest(closest_node,closest_dist,node9->position_);
//    Extend(kd_tree,Q,node9,closest_node,Q->cspace->saturation_delta_,10,Q->cspace->move_goal_);
//    kdTree.row(kdTreePos++) = node9->position_;
//    reduceInconsistency(Q, Q->cspace->move_goal_, Q->cspace->robot_radius_, root, 10);

    std::cout << "\nKD-Tree" << std::endl;
    kd_tree->PrintTree(root);
//    kd_tree->KDFindNearest(closest_node,closest_dist,goal->position_);
//    printRRTxPath(closest_node);

///////////////////

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->cspace->saturation_delta_, p.ball_constant*(
                                pow(log(1+kd_tree->tree_size_)/(kd_tree->tree_size_),
                                    1/Q->cspace->num_dimensions_) ));
        now_time = GetTimeNs(startTime);

        slice_end = (1+slice_counter)*p.slice_time;

        /// Check for warm up time
        if(Q->cspace->in_warmup_time_
                && Q->cspace->warmup_time_ < Q->cspace->time_elapsed_) {
            Q->cspace->in_warmup_time_ = false;
        }

        /// Update Obstacles
        //Obstacle::UpdateObstacles(Q->cspace);


        // Remove obstacles
//        {
//            cout << "locking" << endl;lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
//            list_item = Q->cspace->obstacles->front_;
//        }
//        removed = false;
//        while(list_item != list_item->child_) {
//            obstacle = list_item->obstacle_;

//            if(!obstacle->sensible_obstacle_ && obstacle->obstacle_used_
//                    && (obstacle->start_time_ + obstacle->life_span_
//                        <= Q->cspace->time_elapsed_)) {
//                // Time to remove obstacle
//                cout << "time to remove" << endl;
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
//                removed = true;
//            } else if(obstacle->sensible_obstacle_
//                      && !obstacle->obstacle_used_after_sense_
//                      && (Q->cspace->distanceFunction(prev_pose,
//                                                 obstacle->position_)
//                          < robot_sensor_range + obstacle->radius_)) {
//                // Place to remove obstacle
//                // The space that used to be in this obstacle was never
//                // sampled so there will be a hole in the graph where it used
//                // to be.
//                // So require that the next few samples come from that space
//                cout << "place to remove" << endl;
//                RandomSampleObs(Q->cspace,kd_tree,obstacle);
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
//                obstacle->sensible_obstacle_ = false;
//                obstacle->start_time_ = INF;
//                removed = true;

//            } else if(Q->cspace->space_has_time_
//                      && (obstacle->next_direction_change_time_ > prev_pose(2))
//                      && obstacle->last_direction_change_time_ != prev_pose(2)) {
//                cout << "direction change" << endl;
//                // A moving obstacle with unknown path is changing direction,
//                // so remove its old anticipated trajectory
//                RemoveObstacle(kd_tree,Q,obstacle,root,hyper_ball_rad,
//                               Q->cspace->time_elapsed_,Q->cspace->move_goal_);
//                obstacle->obstacle_used_ = true;
//                removed = true;
//            }
//            list_item = list_item->child_;
//        }
//        if(removed) {
//            cout << "Obstacle Removed" << endl;
//            reduceInconsistency(Q,Q->cspace->move_goal_,Q->cspace->robot_radius_,
//                                root,hyper_ball_rad);
//        }

        // Add Obstacles
        {
            cout << "locking" << endl;lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            list_item = Q->cspace->obstacles_->front_;
            cout << "unlocking" << endl;

        }
        added = false;
        Eigen::VectorXd robot_pose;
        {
            cout << "locking" << endl;lock_guard<mutex> lock(robot->robot_mutex);
            robot_pose = robot->robot_pose;
            cout << "unlocking" << endl;

        }
        while(list_item != list_item->child_) {
            obstacle = list_item->obstacle_;

            if(!obstacle->sensible_obstacle_ && !obstacle->obstacle_used_
                    && obstacle->start_time_ <= Q->cspace->time_elapsed_
                    && Q->cspace->time_elapsed_
                    <= obstacle->start_time_ + obstacle->life_span_) {
                // Time to add
                cout << "time to add" << endl;
                /// Adding this causes assertion failed index >=0 && index < size()
                obstacle->obstacle_used_ = true;
                AddObstacle(kd_tree,Q,obstacle,root);
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
                AddObstacle(kd_tree,Q,obstacle,root);
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
                obstacle->sensible_obstacle_ = false;
                added = true;
            } else if(Q->cspace->space_has_time_
                      && obstacle->next_direction_change_time_ > robot_pose(3)
                      && obstacle->last_direction_change_time_ != robot_pose(3)) {
                // Time that a moving obstacle with unknown path changes
                // direction
                cout << "direction change" << endl;
                obstacle->obstacle_used_ = true;
                obstacle->ChangeObstacleDirection(Q->cspace,robot_pose(3));
                AddObstacle(kd_tree,Q,obstacle,root);
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
                obstacle->last_direction_change_time_ = robot_pose(3);
            } else if(Q->cspace->warmup_time_just_ended_ && obstacle->obstacle_used_) {
                // Warm up time is over, so we need to treat all obstacles
                // as if they have just been added
                cout << "finished warm up time" << endl;
                AddObstacle(kd_tree,Q,obstacle,root);
                // Now check the robot's current move to its target
                if(robot->robot_edge_used && robot->robot_edge->ExplicitEdgeCheck(obstacle))
                    robot->current_move_invalid = true;
                added = true;
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

        Q->cspace->time_elapsed_ = (GetTimeNs(startTime)
                             - Q->cspace->start_time_ns_)/1000000000.0;
        if(Q->cspace->time_elapsed_ >= slice_end) {
            cout << "\nIteration: " << i++ << endl << "---------" << endl;

            slice_start = now_time;
            slice_end = (++slice_counter)*p.slice_time;
            trunc_elapsed_time = floor(Q->cspace->time_elapsed_*1000.0)/1000.0;

            /// Move robot
            if(Q->cspace->time_elapsed_ > p.planning_only_time + p.slice_time) {
                if(p.move_robot_flag) {
                    /*MoveRobot(Q,kd_tree,root,
                              p.slice_time,hyper_ball_rad,robot);*/
                    // Record data (robot path)
                    rHist.row(histPos++) = robot->robot_pose;
                    if( robot->robot_edge != prev_edge) {
                        // Record data (edges)
                        kdEdge.row(kdEdgePos++)
                                = robot->robot_edge->start_node_->position_;
                        kdEdge.row(kdEdgePos++)
                                = robot->robot_edge->end_node_->position_;
                    }
                } else { cout << "robot not moved" << endl; break; }
            }
            prev_edge = robot->robot_edge;

            /// Check for completion
            current_distance = kd_tree->distanceFunction(robot->robot_pose,
                                                        root->position_);
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
        }
    }
    return robot;
}

double distance_function(Eigen::VectorXd a, Eigen::VectorXd b)
{
    Eigen::ArrayXd temp = a.head(2) - b.head(2);
    temp = temp * temp;
    return sqrt( temp.sum()
                 + pow(min(abs(a(2)-b(2)),
                           min(a(2),b(2)) + 2.0*PI
                           - max(a(2),b(2)) ), 2));
}

int main(int argc, char* argv[])
{
    /// C-Space
    int dims = 3;
    double envRad = 10.0;
    Eigen::Vector3d lbound, ubound;
    lbound << -envRad, -envRad, 0.0;
    ubound << envRad, envRad, 2*PI;

    Eigen::Vector3d start, goal;
    start << 0.0,0.0,-3*PI/4;
    goal  << 25.0,25.0,-3*PI/4;

    shared_ptr<ConfigSpace> cspace
            = make_shared<ConfigSpace>(dims,lbound,ubound,start,goal);
    cspace->SetDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1;
    cspace->robot_radius_ = 0.5;
    cspace->robot_velocity_ = 10.0;
    cspace->min_turn_radius_ = 1.0;
    cspace->prob_goal_ = 0.01;
    cspace->space_has_time_ = false;
    cspace->space_has_theta_ = true; // Dubin's model

    /// K-D Tree
    Eigen::VectorXi wv(1);
    wv(0) = 2;
    Eigen::VectorXd wpv(1);
    wpv(0) = 2.0*PI;

    /// Parameters
    string alg_name = "RRTx";
    string obstacle_file = argv[1];
    double plan_time = 0.0;
    double slice_time = 1.0/100;
    double delta = 10.0;
    double ball_const = 100.0;
    double change_thresh = 1.0;
    double goal_thresh = 0.5;
    bool move_robot = true;

    /// Read in Obstacles
    Obstacle::ReadObstaclesFromFile(obstacle_file,cspace);

    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wv, wpv);

    shared_ptr<thread> vis_thread;

    /// Run RRTx
    shared_ptr<RobotData> robot = RRTX(problem,vis_thread);

    /// Save data
    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = robot->robot_move_path.block(0,0,
                                             robot->num_robot_move_points-1,3);
    lastpoints = robot->robot_move_path.block(1,0,
                                            robot->num_robot_move_points-1,3);

    diff = firstpoints - lastpoints;
    diff = diff*diff;
    for(int i = 0; i < diff.rows(); i++) {
        diff.col(0)(i) = diff.row(i).sum();
    }

    double moveLength = diff.col(0).sqrt().sum();
    cout << "Robot traveled: " << moveLength << " units" << endl;

    double totalTime = GetTimeNs(startTime);
    cout << "Total time: " << totalTime/1000000000.0 << " s" << endl;

    ofstream ofs;
    ofs.open("robotPath.txt", ofstream::out);
    for(int j = 0; j < rHist.rows(); j++) {
        ofs << rHist.row(j) << "\n";
    }
    ofs.close();

    ofs.open("kdTree.txt", ofstream::out);
    for(int k = 0; k < kdTree.rows(); k++) {
        ofs << kdTree.row(k) << "\n";
    }
    ofs.close();

    ofs.open("kdEdge.txt", ofstream::out);
    for(int p = 0; p < kdEdge.rows(); p += 2) {
        ofs << kdEdge.row(p) << "\n" << kdEdge.row(p+1) << "\n";
    }
    ofs.close();

    cout<< "Data written to kdTree.txt, kdEdge.txt, and robotPath.txt" <<endl;
    vis_thread->join();

    return 0;
}
