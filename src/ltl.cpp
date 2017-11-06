#include "ltl.h"
#include "../src/list.cpp"

Robot_ptr Ltl(Problem p)
{

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

