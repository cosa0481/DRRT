/* straighttest.cpp
 * Corin Sandford
 * Spring 2017
 * Tests Dubin's model to make sure it
 * plans a straight line.
 */

#include <DRRT/drrt.h> // the RRTx library

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

/// Main control function
shared_ptr<RobotData> RRTX(Problem p)
{
    if(p.search_type != "RRTx") {
        std::cout << "y u no want RRTx?" << std::endl;
        exit(1);
    }

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
    shared_ptr<KDTree> kdtree
            = make_shared<KDTree>(p.c_space->d,p.wraps,p.wrap_points);
    kdtree->setDistanceFunction(p.distance_function);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->S->start);
    //explicitNodeCheck(S,root);
    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;
    root->rrtParentEdge = Edge::newEdge(Q->S,kdtree,root,root);
    root->rrtParentUsed = false;
    kdtree->kdInsert(root);
    kdTree.row(kdTreePos++) = root->position;

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->S->goal);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
//    kdtree->kdInsert(goal);
    kdTree.row(kdTreePos++) = goal->position;

    Q->S->goalNode = goal;
    Q->S->root = root;
    Q->S->moveGoal = goal;
    Q->S->moveGoal->isMoveGoal = true;

    /// Robot
    shared_ptr<RobotData> robot
            = make_shared<RobotData>(Q->S->goal, goal, MAXPATHNODES, Q->S->d);

    if(Q->S->spaceHasTime) {
        addOtherTimesToRoot(Q->S,kdtree,goal,root,Q->type);
    }

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

    double current_distance;
    double move_distance;
    Eigen::Vector3d prev_pose;
    shared_ptr<Edge> prev_edge;

    current_distance = kdtree->distanceFunction(robot->robotPose,
                                                root->position);
    prev_pose = robot->robotPose;

    shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
    shared_ptr<double> closest_dist = make_shared<double>(INF);

///////////////////
    Eigen::Vector3d position;

    //kdtree->printTree(root);

    position(0) = 5;
    position(1) = 5;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node4 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node4->position);
    extend(kdtree,Q,node4,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node4->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    // If the difference between the first node and the root is
    // > delta, then set its LMC to INF so it can be recalculated
    // when the next node is added
    // Noticed this when adding 8,12 before 5,5 and 8,12 would
    // remain the rrtChild of the root instead of switching to 5,5
    if( node4->rrtLMC - node4->rrtParentEdge->endNode->rrtLMC > 10 )
        node4->rrtLMC = INF;

    position(0) = 8;
    position(1) = 12;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node1 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node1->position);
    extend(kdtree,Q,node1,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node1->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 15;
    position(1) = 15;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node5 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node5->position);
    extend(kdtree,Q,node5,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node5->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 7;
    position(1) = 3;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node7 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node7->position);
    extend(kdtree,Q,node7,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node7->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 10;
    position(1) = 10;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node3 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node3->position);
    extend(kdtree,Q,node3,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node3->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 18;
    position(1) = 22;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node10 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node10->position);
    extend(kdtree,Q,node10,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node10->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 22;
    position(1) = 18;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node11 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node11->position);
    extend(kdtree,Q,node11,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node11->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 13;
    position(1) = 17;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node8 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node8->position);
    extend(kdtree,Q,node8,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node8->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 20;
    position(1) = 20;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node12 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node12->position);
    extend(kdtree,Q,node12,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node12->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 3;
    position(1) = 7;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node6 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node6->position);
    extend(kdtree,Q,node6,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node6->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 12;
    position(1) = 8;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node2 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node2->position);
    extend(kdtree,Q,node2,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node2->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    position(0) = 17;
    position(1) = 13;
    position(2) = -3*PI/4;
    shared_ptr<KDTreeNode> node9 = make_shared<KDTreeNode>(position);
    kdtree->kdFindNearest(closest_node,closest_dist,node9->position);
    extend(kdtree,Q,node9,closest_node,Q->S->delta,10,Q->S->moveGoal);
    kdTree.row(kdTreePos++) = node9->position;
    reduceInconsistency(Q, Q->S->moveGoal, Q->S->robotRadius, root, 10);

    std::cout << "\nKD-Tree" << std::endl;
    kdtree->printTree(root);
    kdtree->kdFindNearest(closest_node,closest_dist,goal->position);
    printRRTxPath(closest_node);
///////////////////

    int i = 0;
    while(true) {
        double hyper_ball_rad = min(Q->S->delta, p.ball_constant*(
                                pow(log(1+kdtree->treeSize)/(kdtree->treeSize),
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
                    moveRobot(Q,kdtree,root,
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

            /// Check for completion
            current_distance = kdtree->distanceFunction(robot->robotPose,
                                                        root->position);
            move_distance = kdtree->distanceFunction(robot->robotPose,
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

int main() {
    /// Start and Goal
    Eigen::Vector3d start, goal;
    start << 0.0,0.0,-3*PI/4;
    goal  << 25.0,25.0,-3*PI/4;

    /// C-Space
    int dims = 3;
    double envRad = 50.0;
    Eigen::Vector3d lbound, ubound;
    lbound << -envRad, -envRad, 0.0;
    ubound << envRad, envRad, 2*PI;

    shared_ptr<CSpace> cspace
            = make_shared<CSpace>(dims,lbound,ubound,start,goal);

    cspace->robotRadius = 0.5;
    cspace->robotVelocity = 10.0;
    cspace->minTurningRadius = 1.0;
    cspace->pGoal = 0.01;
    cspace->spaceHasTime = false;
    cspace->spaceHasTheta = true; // Dubin's model

    /// KD-Tree
    Eigen::VectorXi wv(1);
    wv(0) = 2;
    Eigen::VectorXd wpv(1);
    wpv(0) = 2.0*PI;

    /// Parameters
    string alg_name = "RRTx";
    double plan_time = 0.0;
    double slice_time = 1.0/100;
    double delta = 10.0;
    double ball_const = 100.0;
    double change_thresh = 1.0;
    double goal_thresh = 0.5;
    bool move_robot = true;

    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wv, wpv, distance_function);

    /// Run RRTx
    shared_ptr<RobotData> robot = RRTX(problem);

    /// Save data
    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = robot->robotMovePath.block(0,0,
                                             robot->numRobotMovePoints-1,3);
    lastpoints = robot->robotMovePath.block(1,0,
                                            robot->numRobotMovePoints-1,3);

    diff = firstpoints - lastpoints;
    diff = diff*diff;
    for(int i = 0; i < diff.rows(); i++) {
        diff.col(0)(i) = diff.row(i).sum();
    }

    double moveLength = diff.col(0).sqrt().sum();
    cout << "Robot traveled: " << moveLength << " units" << endl;

    double totalTime = getTimeNs(startTime);
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

    return 0;
}
