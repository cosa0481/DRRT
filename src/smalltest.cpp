/* smalltest.cpp
 * Corin Sandford
 * Spring 2017
 * Used to test high level things in the
 * library on a smaller scale than test.cpp
 */

#include <DRRT/visualizer.h>
#include <DRRT/thetastar.h>
#include <DRRT/mainloop.h>
#include <DRRT/moverobot.h>

using namespace std;

bool timingex = false;

chrono::time_point<chrono::high_resolution_clock> start_time;

// Takes path from Theta* and returns avg theta of each line for theta bias
vector<double> PathToThetas(vector<Eigen::VectorXd> path)
{
    vector<double> thetas;
    double angle, x = 0, y = 0;
    Eigen::VectorXd current_point, prev_point;
    cout << "Any-Angle Path:" << endl;
    for( int i = 0; i < path.size(); i++ ) {
        cout << path.at(i) << endl << endl;
        if(i==0) continue;
        current_point = path.at(i);
        prev_point = path.at(i-1);
//        path_a = (current_point(1)-prev_point(1))
//                /(current_point(0)-prev_point(0));
//        path_b = -1;
//        path_c = min(abs(current_point(1)),abs(prev_point(1)));
//        lines.insert(lines.begin(), Eigen::Vector3d(path_a,path_b,path_c));
        angle = atan2(prev_point(1)-current_point(1),
                      prev_point(0)-current_point(0));
        x = cos(angle);
        y = sin(angle);
        thetas.push_back(atan2(y,x));
    }
    return thetas;
}

/// AlGORITHM CONTROL FUNCTION
// This function runs RRTx with the parameters defined in main()
shared_ptr<RobotData> Rrtx(Problem p, shared_ptr<thread> &vis)
{
    /// Initialize

    // Queue
    shared_ptr<Queue> Q = make_shared<Queue>();
    Q->priority_queue = make_shared<BinaryHeap>(false); // priority queue use Q functions
    Q->obs_successors = make_shared<JList>(true); // obstacle stack uses KDTreeNodes
    Q->change_thresh = p.change_threshold;
    Q->type = p.search_type;
    Q->cspace = p.c_space;
    Q->cspace->sample_stack_ = make_shared<JList>(true); // uses KDTreeNodes
    Q->cspace->saturation_delta_ = p.delta;

    // K-D Tree
    shared_ptr<KDTree> kd_tree
            = make_shared<KDTree>(Q->cspace->num_dimensions_,p.wraps,p.wrap_points);
    kd_tree->SetDistanceFunction(Q->cspace->distanceFunction);

    shared_ptr<KDTreeNode> root = make_shared<KDTreeNode>(Q->cspace->start_);
    ExplicitNodeCheck(Q,root);
    root->rrt_tree_cost_ = 0.0;
    root->rrt_LMC_ = 0.0;
    root->rrt_parent_edge_ = Edge::NewEdge(Q->cspace,kd_tree,root,root);
    root->rrt_parent_used_ = false;
    kd_tree->KDInsert(root);

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>(Q->cspace->goal_);
    goal->rrt_tree_cost_ = INF;
    goal->rrt_LMC_ = INF;

    Q->cspace->goal_node_ = goal;
    Q->cspace->root_ = root;
    Q->cspace->move_goal_ = goal;
    Q->cspace->move_goal_->is_move_goal_ = true;

    // Robot
    shared_ptr<RobotData> robot
            = make_shared<RobotData>(Q->cspace->goal_, goal,
                                     MAXPATHNODES, Q->cspace->num_dimensions_);
    robot->robot_sensor_range = 20.0;

    if(Q->cspace->space_has_time_) {
        AddOtherTimesToRoot(Q->cspace,kd_tree,goal,root,Q->type);
    }

    // Any-Angle Path
    vector<Eigen::VectorXd> path = ThetaStar(Q);
//    path.push_back(Q->cspace->goal_);
    robot->best_any_angle_path = path;
    vector<double> thetas = PathToThetas(path); // prints the path

    /// End Initialization

    start_time = chrono::high_resolution_clock::now();
    Q->cspace->time_elapsed_ = 0.0;
    Q->cspace->start_time_ns_
            = chrono::duration_cast<chrono::nanoseconds>(start_time
                                                         -start_time).count();

    // Initiate threads
    vis = make_shared<thread>(visualizer, kd_tree, robot, Q);
    cout << "Started Visualizer Thread" << endl;

    thread obstacle_management = thread(CheckObstacles, Q, kd_tree, robot,
                                        p.ball_constant);
    cout << "Started Obstacle Thread" << endl;

    vector<thread> thread_pool(p.num_threads);
    int threads = 1;
    for(auto & thr : thread_pool) {
        thr = thread(RrtMainLoop, Q, kd_tree, robot, start_time,
                     p.ball_constant, p.slice_time, thetas, path);
        cout << "Started Main Loop Thread " << threads++ << endl;
    }

    thread move_robot = thread(RobotMovement, Q, kd_tree, robot,
                               p.planning_only_time, p.slice_time,
                               p.goal_threshold, p.ball_constant);
    cout << "Started Robot Movement Thread" << endl;

    move_robot.join();
    cout << "Joined Robot Movement Thread" << endl;

    obstacle_management.join();
    cout << "Joined Obstacle Thread" << endl;

    threads = 1;
    for(auto & thr : thread_pool) {
        thr.join();
        cout << "Joined Main Loop Thread " << threads++ << endl;
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
    goal << 49.0, 49.0, -3*PI/4;    // where the robot begins

    shared_ptr<ConfigSpace> cspace
            = make_shared<ConfigSpace>(dims,lbound,ubound,start,goal);
    cspace->SetDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1;
    cspace->robot_radius_ = 0.5;
    cspace->robot_velocity_ = 20.0;
    cspace->min_turn_radius_ = 1.0;
    cspace->prob_goal_ = 0.01;           // probability of sampling the goal node
    cspace->space_has_time_ = false;
    cspace->space_has_theta_ = true;   // Dubin's car model

    /// K-D Tree
    // Dubin's model wraps_ theta (4th entry) at 2pi
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
    int num_threads = 10;           // number of main loop threads to spawn

    /// Read in Obstacles
    Obstacle::ReadObstaclesFromFile(obstacle_file, cspace);

    // Create a new problem for RRTx
    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wrap_vec, wrap_points_vec,
                              num_threads);

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
    double totalTime = GetTimeNs(start_time);
    cout << "Total time: " << totalTime/1000000000.0
              << " s" << endl;

    vis_thread->join();

    return 0;
}
