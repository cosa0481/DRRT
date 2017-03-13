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
        leaf = leaf->rrtParentEdge->endNode;
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
    Q->Q = make_shared<BinaryHeap>(false); // priority queue use Q functions
    Q->OS = make_shared<JList>(true); // obstacle stack uses KDTreeNodes
    Q->changeThresh = p.change_threshold;
    Q->type = p.search_type;
    Q->S = p.c_space;
    Q->S->sampleStack = make_shared<JList>(true); // uses KDTreeNodes
    Q->S->delta = p.delta;

    /// KD-Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(p.c_space->d,p.wraps,p.wrap_points);
    kd_tree->setDistanceFunction(p.distance_function);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->S->start);
    //explicitNodeCheck(S,root);
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

    shared_ptr<thread> vis_thread = make_shared<thread>(visualizer,kd_tree,robot);
    vis = vis_thread;

    /// End Initialization

    /// Main loop
    startTime = chrono::high_resolution_clock::now();
    double slice_counter = 0;
    double slice_start
     = chrono::duration_cast<chrono::nanoseconds>(startTime-startTime).count();
    Q->S->startTimeNs = slice_start;
    Q->S->timeElapsed = 0.0;
    double slice_end;

    double now_time = getTimeNs(startTime);
    double trunc_elapsed_time;

    double old_rrtLMC, current_distance, move_distance, this_dist;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;

    current_distance = kd_tree->distanceFunction(robot->robotPose,
                                                root->position);
    prev_pose = robot->robotPose;

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->S->delta, p.ball_constant*(
                                pow(log(1+kd_tree->treeSize)/(kd_tree->treeSize),
                                    1/Q->S->d) ));
        now_time = getTimeNs(startTime);

        slice_end = (1+slice_counter)*p.slice_time;

        /// Check for warm up time
        if(Q->S->inWarmupTime
                && Q->S->warmupTime < Q->S->timeElapsed) {
            Q->S->inWarmupTime = false;
        }

        Q->S->timeElapsed = (getTimeNs(startTime)
                             - Q->S->startTimeNs)/1000000000.0;
        if(Q->S->timeElapsed >= slice_end) {
            cout << "\nIteration: " << i++ << endl << "---------" << endl;

            slice_start = now_time;
            slice_end = (++slice_counter)*p.slice_time;
            trunc_elapsed_time = floor(Q->S->timeElapsed*1000.0)/1000.0;

            /// Move robot
            if(Q->S->timeElapsed > p.planning_only_time + p.slice_time) {
                if(p.move_robot_flag) {
                    moveRobot(Q,kd_tree,root,
                              p.slice_time,hyper_ball_rad,robot);
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
            reduceInconsistency(Q,Q->S->moveGoal, Q->S->robotRadius,
                                root, hyper_ball_rad);
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

            if( i == 1 ) { printRRTxPath(robot->nextMoveTarget); }
            if( !robot->moving ) {
                /// Sample free space
                shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
                shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
                shared_ptr<double> closest_dist = make_shared<double>(INF);

                new_node = randNodeOrFromStack(Q->S);
                if(new_node->kdInTree) continue;

                kd_tree->kdFindNearest(closest_node,closest_dist,
                                      new_node->position);

                /// Saturate new node
                this_dist = kd_tree->distanceFunction(new_node->position,
                                                        closest_node->position);
                if(this_dist > Q->S->delta && new_node != Q->S->goalNode) {
                    Edge::saturate(new_node->position, closest_node->position,
                                   Q->S->delta, this_dist);
                }

                /// Check for obstacles
                //bool explicitly_unsafe;
                //explicitNodeCheck(explicitly_unsafe,Q->S,new_node)
                //if(explicitly_unsafe) continue;

                /// Extend graph
                if(extend(kd_tree,Q,new_node,closest_node,
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
            }
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

    shared_ptr<CSpace> cspace
            = make_shared<CSpace>(dims,lbound,ubound,start,goal);

    cspace->robotRadius = 0.5;
    cspace->robotVelocity = 10.0;
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
                              move_robot, wrap_vec, wrap_points_vec,
                              distance_function);

    // Visualizer thread to join
    shared_ptr<thread> visualizer_thread;

    /// Run RRTx
    shared_ptr<RobotData> robot_data = RRTX(problem,visualizer_thread);

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
    visualizer_thread->join();

    return 0;
}
