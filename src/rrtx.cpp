#include "rrtx.h"
#include "../src/list.cpp"

using namespace std;

Robot_ptr Rrtx(Problem p)
{
    CSpace_ptr cspace = p.cspace;

    // Make K-D tree
    cspace->kdtree_ = make_shared<KdTree>(NUM_DIM, p.wraps, p.wrap_points);

    Kdnode_ptr root = make_shared<Kdnode>(cspace->start_);
    root->SetLmc(0.0);
    root->SetCost(0.0);
    root->SetRrtParentEdge(Edge::NewEdge(root,root));
    root->SetRrtParentExist(false);
    cspace->kdtree_->Insert(root);

    Kdnode_ptr goal = make_shared<Kdnode>(cspace->end_);
    goal->SetLmc(INF);
    goal->SetCost(INF);

    cspace->goal_node_ = goal;
    cspace->root_node_ = root;
    cspace->move_goal_ = goal;
    cspace->move_goal_->SetIsGoal(true);

    cout << "Created K-D Tree" << endl;

    // Make robot
    cspace->robot_ = make_shared<RobotData>("Dubins Car", cspace->start_, cspace->goal_node_);
    cspace->robot_->robot_sensor_range = 5.0;
    cspace->robot_->radius = 0.5;

    cout << "Created robot: " << cspace->robot_->name << endl;

    // Check root node for feasibility
    if(NodeCheck(cspace, root)) {
        if(DEBUG) cout << "Start node does not pass NodeCheck()" << endl;
        exit(-1);
    }

    // End initialization
    cspace->start_time_ = chrono::high_resolution_clock::now();
    cspace->elapsed_time_ = 0.0;

    // Create threads
    thread obstacle_thread = thread(Obstacle::UpdateObstacles, cspace);
    cout << "Started Obstacle Thread" << endl;

    vector<thread> main_thread_pool(p.threads);
    for(auto & thr : main_thread_pool)
        thr = thread(Rrt, cspace);
    cout << "Started " << main_thread_pool.size() << " Main Loop Threads" << endl;

    thread movement_thread = thread(MovementThread, cspace);
    cout << "Started Movement Thread" << endl;

    cout << "===========================================" << endl;

    movement_thread.join();
    cout << "Joined Movement Thread" << endl;

    for(auto & thr : main_thread_pool)
        thr.join();
    cout << "Joined " << main_thread_pool.size() << " Main Loop Threads" << endl;

    obstacle_thread.join();
    cout << "Joined Obstacle Thread" << endl;

    return cspace->robot_;
}

// For reading polygons from a file
void ReadObstaclesFromFile(std::string obs_file, CSpace_ptr cspace)
{
    ifstream read_stream;
    string line, substring;
    stringstream line_stream;
    int num_polygons = 0;
    int num_points;
    int num_moves;
    read_stream.open(obs_file);
    if(read_stream.is_open()) {
        std::cout << "Obstacle File: " << obs_file << std::endl;
        // Get number of obstacles in file
        getline(read_stream, line);
        num_polygons = stoi(line);
        if(DEBUG) std::cout << "Number of polygons: " << num_polygons << std::endl;
        line = "";
        for(int i = 0; i < num_polygons; i++) {
            // Read through first delimiter line
            getline(read_stream, line);
            line = "";

            // Get origin center point
            Eigen::Vector3d origin;
            getline(read_stream, line);
            line_stream = stringstream(line);
            getline(line_stream, substring, ',');
            origin(0) = stod(substring);
            getline(line_stream, substring);
            origin(1) = stod(substring);
            origin(2) = 0.0;
            substring = "";
            line = "";

            // Get moves
            getline(read_stream, line);
            num_moves = stoi(line);
            Eigen::MatrixXd path;
            Eigen::VectorXd path_times;
            path.resize(num_moves, NUM_DIM);
            path_times.resize(num_moves);
            for(int j = 0; j < num_moves; j++) {
                getline(read_stream, line);
                path_times(j) = stod(line);
                line = "";
            }
            Eigen::Vector3d new_origin;
            for(int k = 0; k < num_moves; k++) {
                getline(read_stream, line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                new_origin(0) = stod(substring);
                getline(line_stream, substring);
                new_origin(1) = stod(substring);
                substring = "";
                new_origin(2) = 0.0;
                path.row(k)(0) = new_origin(0);
                path.row(k)(1) = new_origin(1);
                path.row(k)(2) = new_origin(2);
            }

            // Get number of points in obstacle polygon
            getline(read_stream, line);
            num_points = stoi(line);
//            if(DEBUG) std::cout << "Number of points in polygon " << i+1 << ": " << num_points << std::endl;
            line = "";
            Eigen::MatrixXd polygon;
            if(num_points > 0) polygon.resize(num_points, 2);
            for(int p = 0; p < num_points; p++) {
                getline(read_stream, line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                polygon.row(p)(0) = stod(substring);
                getline(line_stream, substring);
                polygon.row(p)(1) = stod(substring);
                substring = "";
                line = "";
            }

            // Create an obstacle object from this information
            Obstacle_ptr new_obstacle
                    = std::make_shared<Obstacle>(3, origin, polygon, path, path_times, cspace);

            // Add new obstacle to Bullet for collision detection
            std::shared_ptr<btCollisionObject> coll_obj = std::make_shared<btCollisionObject>();
            std::shared_ptr<btConvexHullShape> coll_shape = std::make_shared<btConvexHullShape>();
            Eigen::MatrixX2d shape = new_obstacle->GetShape().GetRegion();
            for(int q = 0; q < shape.rows(); q++) {
                coll_shape->addPoint(
                            btVector3((btScalar) shape(q,0),
                                      (btScalar) shape(q,1),
                                      (btScalar) -0.1));
                coll_shape->addPoint(
                            btVector3((btScalar) shape(q,0),
                                      (btScalar) shape(q,1),
                                      (btScalar) 0.1));
            }
            coll_obj->setCollisionShape(coll_shape.get());
            cspace->bt_collision_world_->addCollisionObject(coll_obj.get());
            new_obstacle->SetCollisionObject(coll_obj);
            new_obstacle->SetCollisionShape(coll_shape);
            new_obstacle->AddToCSpace();
        }
    } else { if(DEBUG) std::cout << "Error opening obstacle file: " << obs_file << std::endl; }
    read_stream.close();
    std::cout << "Read in " << num_polygons << " obstacles" << std::endl;
}
