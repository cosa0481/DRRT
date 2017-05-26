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
bool debug_bullet = false;

chrono::time_point<chrono::high_resolution_clock> start_time;

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
                                     Q->cspace->num_dimensions_);
    robot->robot_sensor_range = 5.0;

    if(Q->cspace->space_has_time_) {
        AddOtherTimesToRoot(Q->cspace,kd_tree,goal,root,Q->type);
    }

    // Initiate threads
    vis = make_shared<thread>(visualizer, kd_tree, robot, Q);
    cout << "Started Visualizer Thread" << endl;


    if(debug_bullet) {
        int grid_size = 50;
        double gran = 1;
        for(double i = -1; i < grid_size; i = i + gran) {
            for(double j = -1; j < grid_size; j = j + gran) {
                // +,0
                shared_ptr<KDTreeNode> node1 = make_shared<KDTreeNode>(Eigen::Vector3d(i,j,0));
                shared_ptr<KDTreeNode> node2 = make_shared<KDTreeNode>(Eigen::Vector3d(i+gran,j,0));
                shared_ptr<Edge> edge = Edge::NewEdge(Q->cspace, kd_tree, node1, node2);
                edge->CalculateTrajectory();
                ExplicitEdgeCheck(Q->cspace,edge);

                // 0,+
                node1 = make_shared<KDTreeNode>(Eigen::Vector3d(i,j,PI/2));
                node2 = make_shared<KDTreeNode>(Eigen::Vector3d(i,j+gran,PI/2));
                edge = Edge::NewEdge(Q->cspace, kd_tree, node1, node2);
                edge->CalculateTrajectory();
                ExplicitEdgeCheck(Q->cspace,edge);

                // +,+
                node1 = make_shared<KDTreeNode>(Eigen::Vector3d(i,j,PI/4));
                node2 = make_shared<KDTreeNode>(Eigen::Vector3d(i+gran,j+gran,PI/4));
                edge = Edge::NewEdge(Q->cspace, kd_tree, node1, node2);
                edge->CalculateTrajectory();
                ExplicitEdgeCheck(Q->cspace,edge);

                // -,+
                node1 = make_shared<KDTreeNode>(Eigen::Vector3d(i,j,3*PI/4));
                node2 = make_shared<KDTreeNode>(Eigen::Vector3d(i-gran,j+gran,3*PI/4));
                edge = Edge::NewEdge(Q->cspace, kd_tree, node1, node2);
                edge->CalculateTrajectory();
                ExplicitEdgeCheck(Q->cspace,edge);
            }
        }
        cout << "collision minefield completed" << endl;
        vis->join();
        exit(0);
    }

    // Any-Angle Path
    robot->best_any_angle_path = ThetaStar(Q);
    robot->thetas = PathToThetas(robot->best_any_angle_path);

    /// End Initialization

    start_time = chrono::high_resolution_clock::now();
    Q->cspace->time_elapsed_ = 0.0;
    Q->cspace->start_time_ns_
            = chrono::duration_cast<chrono::nanoseconds>(start_time
                                                         -start_time).count();

    thread obstacle_management = thread(CheckObstacles, Q, kd_tree, robot,
                                        p.ball_constant);
    cout << "Started Obstacle Thread" << endl;

    vector<thread> thread_pool(p.num_threads);
    int threads = 1;
    for(auto & thr : thread_pool) {
        thr = thread(RrtMainLoop, Q, kd_tree, robot, start_time,
                     p.ball_constant, p.slice_time);
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

    //PrintRrtxPath(Q->cspace->goal_node_);
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
    start << 0.0, 0.0, -3*PI/4;     // robot destination
    goal << 49.0, 49.0, -3*PI/4;    // where the robot begins

    shared_ptr<ConfigSpace> cspace
            = make_shared<ConfigSpace>(dims,lbound,ubound,start,goal);
    cspace->SetDistanceFunction(distance_function);

    cspace->obs_delta_ = -1.0;
    cspace->collision_distance_ = 0.1; // distance to be considered a collision
    cspace->robot_radius_ = 0.5;       // robot radius
    cspace->robot_velocity_ = 20.0;    // robot velocity
    cspace->min_turn_radius_ = 1.0;    // robot minimum turn radius
    cspace->prob_goal_ = 0.01;         // probability of sampling the goal node
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
    double plan_time = 30.0;        // plan *ONLY* for this long
    double slice_time = 1.0/100;    // iteration time limit
    double delta = 5.0;             // distance between graph nodes
    double ball_const = 100.0;      // search d-ball radius (10.0 worked)
    double change_thresh = 1.0;     // node change detection
    double goal_thresh = 0.5;       // goal detection
    bool move_robot = true;         // move robot after plan_time/slice_time
    int num_threads = 20;            // number of main loop threads to spawn (3)

    /// Read in Obstacles
    Obstacle::ReadObstaclesFromFile(obstacle_file, cspace);

    // Create a new problem for RRTx
    Problem problem = Problem(alg_name, cspace, plan_time, slice_time, delta,
                              ball_const, change_thresh, goal_thresh,
                              move_robot, wrap_vec, wrap_points_vec,
                              num_threads);

    // Pointer to visualizer thread (created in RRTX())
    shared_ptr<thread> vis_thread;

    chrono::steady_clock::time_point start_alg = chrono::steady_clock::now();

    /// Run RRTx
    shared_ptr<RobotData> robot_data = Rrtx(problem, vis_thread);

    chrono::steady_clock::time_point end_alg = chrono::steady_clock::now();

    // Calculate and display time elapsed
    double total_time = chrono::duration_cast<chrono::duration<double>>
                        (end_alg - start_alg).count();
    cout << "\nTotal time: " << total_time << " s" << endl;

    vis_thread->join();

    // Delete bullet pointers
    delete cspace->bt_collision_configuration_;
    delete cspace->bt_dispatcher_;
    delete cspace->bt_broadphase_;
    delete cspace->bt_collision_world_;

    return 0;
}
