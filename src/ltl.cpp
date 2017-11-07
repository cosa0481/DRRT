#include "ltl.h"
#include "../src/list.cpp"

Robot_ptr Ltl(Problem p)
{
    CSpace_ptr cspace = p.cspace;

    // K-D Tree
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

    cspace->root_node_ = root;
    cspace->goal_node_ = goal;
    cspace->move_goal_ = goal;
    cspace->move_goal_->SetIsGoal(true);

    cout << "Created K-D Tree" << endl;

    // Robot
    cspace->robot_ = make_shared<RobotData>("Dubinss LTL Car",
                                            cspace->goal_node_->GetPosition(),
                                            cspace->root_node_);
    cspace->robot_->robot_sensor_range = 5.0;
    cspace->robot_->radius = 0.5;

    cout << "Created Robot" << endl;

    if(NodeCheck(cspace, root)) {
        cout << "Start node does not pass NodeCheck()" << endl;
        exit(-2);
    }

    // Visualizer
    thread visualizer_thread = thread(Visualizer, cspace);
    thread obstacle_thread = thread(Obstacle::UpdateObstacles, cspace);

    cout << "Started visualizer and obstacle threads" << endl;

    // Triangulate environment
    MatrixX6d triangles;
    {
        lockguard lock(cspace->mutex_);
        triangles = TriangulatePolygon(cspace->drivable_region_);
    }

    cout << "Triangulated environment" << endl;

    // Display triangulation
    Edge_ptr tri1, tri2, tri3;
    for(int i = 0; i < triangles.rows(); i++) {
        tri1 = Edge::NewEdge(Eigen::Vector3d(triangles(i, 0), triangles(i, 1), 0),
                             Eigen::Vector3d(triangles(i, 2), triangles(i, 3), 0));
        tri2 = Edge::NewEdge(Eigen::Vector3d(triangles(i, 2), triangles(i, 3), 0),
                             Eigen::Vector3d(triangles(i, 4), triangles(i, 5), 0));
        tri2 = Edge::NewEdge(Eigen::Vector3d(triangles(i, 4), triangles(i, 5), 0),
                             Eigen::Vector3d(triangles(i, 0), triangles(i, 1), 0));
        cspace->AddEdgeToVis(tri1);
        cspace->AddEdgeToVis(tri2);
        cspace->AddEdgeToVis(tri3);
    }

    cout << "Added triangulation to visualizer" << endl;
    this_thread::sleep_for(chrono::milliseconds(10000));
    visualizer_thread.join();
    exit(-3);

    // Run Theta*
    {
        lockguard lock(cspace->robot_->mutex);
        cspace->robot_->theta_star_path = ThetaStar(cspace);
        cspace->robot_->thetas = PathToThetas(cspace->robot_->theta_star_path);
    }

    // Find triangle trajectory
    Eigen::Matrix2Xd triangle_traj; // ccw


    // Define sampling region (ccw)
    cspace->drivable_region_ = Region(triangle_traj);

    // Run RRTx in this region
    cspace->start_time_ = chrono::high_resolution_clock::now();

    vector<thread> main_thread_pool(p.threads);
    for(auto & thr : main_thread_pool)
        thr = thread(Rrt, cspace);
    thread movement_thread = thread(MovementThread, cspace);

    movement_thread.join();
    for(auto & thr : main_thread_pool)
        thr.join();
    obstacle_thread.join();
    visualizer_thread.join();

    return cspace->robot_;
}

// For reading static polygons from a file
void ReadStaticObstaclesFromFile(std::string obs_file, CSpace_ptr cspace)
{
    ifstream read_stream;
    string line, substring;
    stringstream line_stream;
    int num_polygons = 0;
    int num_points;
    read_stream.open(obs_file);
    if(read_stream.is_open()) {
        std::cout << "Obstacle File: " << obs_file << std::endl;
        // Get number of obstacles in file
        getline(read_stream, line);
        num_polygons = stoi(line);
//        std::cout << "Number of polygons: " << num_polygons << std::endl;
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

            // Get number of points in obstacle polygon
            getline(read_stream, line);
            num_points = stoi(line);
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
                    = std::make_shared<Obstacle>(3, origin, polygon, cspace);

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
            btQuaternion quat = btQuaternion(btScalar(0.0), btScalar(0.0), btScalar(0.0));
            btVector3 pos = btVector3(btScalar(origin(0)), btScalar(origin(1)), btScalar(0.0));
            coll_obj->setWorldTransform(btTransform(quat, pos));
            cspace->bt_collision_world_->addCollisionObject(coll_obj.get());
            new_obstacle->SetCollisionObject(coll_obj);
            new_obstacle->SetCollisionShape(coll_shape);
            new_obstacle->AddToCSpace();
        }
    } else { if(DEBUG) std::cout << "Error opening obstacle file: " << obs_file << std::endl; }
    read_stream.close();
    std::cout << "Read in " << num_polygons << " obstacles" << std::endl;
}

