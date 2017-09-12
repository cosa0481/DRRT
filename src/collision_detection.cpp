#include <DRRT/collision_detection.h>
#include <DRRT/data_structures.h>

KdnodeList_ptr FindPointsInConflictWithObstacle(KdTree_ptr tree, Obstacle_ptr obs)
{
    KdnodeList_ptr node_list;
    if(NUM_DIM == 3) {
        Eigen::Vector3d obs_center(obs->GetOrigin()(0), obs->GetOrigin()(1), PI);
        node_list = tree->FindWithinRange(obs->GetRadius(), obs_center);
    } else {
        std::cout << "Error: FindPointsInConflictWithObstacle not implemented." << std::endl;
        node_list = std::make_shared<KdnodeList>();
    }
    return node_list;
}

// Detect collision with obs in bullet world for straight line from start to end
bool DetectCollision(Obstacle_ptr obs, Eigen::VectorXd start, Eigen::VectorXd end)
{
    btCollisionObject* segment = new btCollisionObject();

    // Get length of segment start - end
    double x = end(0) - start(0);
    double y = end(1) - start(1);
    double length = sqrt((x*x) + (y*y));

    // Create box around segment with width: 2*robot_radius
    // and height: length + 2*robot_radius
    btCylinderShape* segment_shape = new btCylinderShape(
                btVector3((btScalar) obs->cspace->robot_radius_ + length/2,
                          (btScalar) 0.1,
                          (btScalar) obs->cspace->robot_radius_ + length/2));

    // Set the collision shape of the segment collision object
    segment->setCollisionShape(segment_shape);

    // Add collision object to the collision world
    obs->cspace->bt_collision_world_->addCollisionObject(segment);

    // Set origin to be senter of the line segment
    segment->getWorldTransform().setOrigin(
                btVector3((btScalar) (start(0) + end(0))/2,
                          (btScalar) (start(1) + end(1))/2,
                          (btScalar) 0.0));

    // Find the yaw rotation of the collision object
    double angle = std::atan2(end(1) - start(1), end(0) - start(0));
    x = std::cos(angle);
    y = std::sin(angle);  // was cosine (this may have been an error)
    segment->getWorldTransform().setRotation(
                btQuaternion((btScalar) 0,
                             (btScalar) 0,
                             (btScalar) std::atan2(y,x)));

    // Check for collisions
    obs->cspace->bt_collision_world_->performDiscreteCollisionDetection();

    // Number of manifolds is the number of collisions
    int num_manifolds = obs->cspace->bt_collision_world_->getDispatcher()->getNumManifolds();

    vector<int> num_contacts(num_manifolds);
    std::shared_ptr<btPersistentManifold> contact_manifold;
    const btCollisionObject* obA;
    const btCollisionObject* obB;
    for(int i = 0; i < num_manifolds; i++) {
        contact_manifold = std::make_shared<btPersistentManifold>(
                    *obs->cspace->bt_collision_world_->getDispatcher()
                    ->getManifoldByIndexInternal(i));
        obA = contact_manifold->getBody0();
        obB = contact_manifold->getBody1();
        contact_manifold->refreshContactPoints(obA->getWorldTransform(),
                                               obB->getWorldTransform());
        num_contacts[i] = contact_manifold->getNumContacts();
    }

    // Remove the segment collision object from the collision world
    obs->cspace->bt_collision_world_->removeCollisionObject(segment);

    // If there are any manifolds then there is at least 1 collision
    for(int j = 0; j < num_manifolds; j++) {
        if(num_contacts[j] > 0) {
            return true;
        }
    }
    return false;
}

bool LineCheck(Kdnode_ptr node1, Kdnode_ptr node2, std::shared_ptr<ConfigSpace> cspace)
{
    double saved_theta1 = node1->GetPosition()(2);
    double saved_theta2 = node2->GetPosition()(2);

    // Calculate angle between nodes
    double theta = std::atan2(node2->GetPosition()(1) - node1->GetPosition()(1),
                              node2->GetPosition()(0) - node1->GetPosition()(0));

    // Make nodes point in same direction
    Eigen::Vector3d new_position;
    new_position(0) = node1->GetPosition()(0);
    new_position(1) = node1->GetPosition()(1);
    new_position(2) = theta;
    node1->SetPosition(new_position);
    new_position(0) = node2->GetPosition()(0);
    new_position(1) = node2->GetPosition()(1);
    new_position(2) = theta;
    node2->SetPosition(new_position);

    // Find trajectory between nodes (straight line)
    Edge_ptr edge = Edge::NewEdge(node1, node2);
    double traj_length = 50;
    Eigen::VectorXd x_traj = Eigen::VectorXd::Zero(traj_length + 1);
    Eigen::VectorXd y_traj = Eigen::VectorXd::Zero(traj_length + 1);
    double x_val = node1->GetPosition()(0);
    double y_val = node1->GetPosition()(1);
    double x_dist = std::abs(node2->GetPosition()(0) - x_val);
    double y_dist = std::abs(node2->GetPosition()(1) - y_val);
    int i = 0;
    while(i < traj_length) {
        x_traj(i) = x_val;
        if(node2->GetPosition()(0) > node1->GetPosition()(0))
            x_val += x_dist/traj_length;
        else x_val -= x_dist/traj_length;

        y_traj(i) = y_val;
        if(node2->GetPosition()(1) > node1->GetPosition()(1))
            y_val += y_dist/traj_length;
        else y_val -= y_dist/traj_length;
        i++;
    }

    x_traj(i) = x_val;
    y_traj(i) = y_val;
    Eigen::MatrixXd traj = edge->GetTrajectory();
    traj.resize(traj_length + 1, 2);
    traj.col(0) = x_traj;
    traj.col(1) = y_traj;
    edge->SetTrajectory(traj);

    // Check if this straight line edge is valid
    bool unsafe = EdgeCheck(cspace, edge);

    // Reset original theta values
    new_position(0) = node1->GetPosition()(0);
    new_position(1) = node1->GetPosition()(1);
    new_position(2) = saved_theta1;
    node1->SetPosition(new_position);
    new_position(0) = node2->GetPosition()(0);
    new_position(1) = node2->GetPosition()(1);
    new_position(2) = saved_theta2;
    node2->SetPosition(new_position);

    return unsafe;
}

bool NodeCheck(std::shared_ptr<ConfigSpace> cspace, Kdnode_ptr node)
{
    return PointCheck(cspace, node->GetPosition());
}

bool PointCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point)
{
    if(cspace->in_warmup_time_) return false;
    if(QuickCheck(cspace, point)) return true;

    // Point is not inside any obstacles but still may be in collision
    // b/c of the cspace->robot_radius_
    ObstacleListNode_ptr obs_listnode;
    Obstacle_ptr obstacle;
    int length;
    {
        lockguard lock(cspace->cspace_mutex_);
        obs_listnode = cspace->obstacles_->GetFront();
        length = cspace->obstacles_->GetLength();
        for(int i = 0; i < length; i++) {
            obs_listnode->GetData(obstacle);
            if(PointCheck2D(cspace, point, obstacle)) return true;
            obs_listnode = obs_listnode->GetChild();
        }
    }
    return false;
}

bool PointCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point,
                  Obstacle_ptr obs)
{
    double robot_radius = cspace->robot_radius_;
    double min_dist = cspace->collision_distance_;
    double this_dist = INF;

    if(!obs->IsUsed() || obs->GetLifeTime() <= 0) return false;

    // Calculate distance from robot boundary to obstacle center
    this_dist = DistanceFunction(obs->GetOrigin(), point);
    if(this_dist - obs->GetRadius() > min_dist) return false;

    if(PointInPolygon(point.head(2), obs->GetShape())) return true;

    this_dist = sqrt(DistToPolygonSqrd(point, obs->GetShape()))
            - robot_radius;
    if(this_dist < 0.0) return true;

    return false;
}

bool EdgeCheck(Obstacle_ptr obstacle, Edge_ptr edge)
{
    Eigen::MatrixX2d trajectory = edge->GetTrajectory();
    for(int i = 1; i < trajectory.rows(); i++) {
        if(DetectCollision(obstacle,
                           trajectory.row(i-1),
                           trajectory.row(i)))
            return true;
    }
    return false;
}

bool EdgeCheck(std::shared_ptr<ConfigSpace> cspace, Edge_ptr edge)
{
    if(cspace->in_warmup_time_) return false;

    ObstacleListNode_ptr obs_listnode;
    Obstacle_ptr obstacle;
    int length;
    {
        lockguard lock(cspace->cspace_mutex_);
        obs_listnode = cspace->obstacles_->GetFront();
        length = cspace->obstacles_->GetLength();
        for(int i = 0; i < length; i++) {
            obs_listnode->GetData(obstacle);
            if(EdgeCheck(obstacle, edge)) return true;
            obs_listnode = obs_listnode->GetChild();
        }
    }
    return false;
}

bool QuickCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point)
{
    ObstacleListNode_ptr obs_listnode;
    Obstacle_ptr obstacle;
    int length;
    {
        lockguard lock(cspace->cspace_mutex_);
        obs_listnode = cspace->obstacles_->GetFront();
        length = cspace->obstacles_->GetLength();
        for(int i = 0; i < length; i++) {
            obs_listnode->GetData(obstacle);
            if(QuickCheck2D(cspace, point, obstacle)) return true;
            obs_listnode = obs_listnode->GetChild();
        }
    }
    return false;
}

bool QuickCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point,
                  Obstacle_ptr obs)
{
    if(!obs->IsUsed() || obs->GetLifeTime() <= 0) return false;
    if(EuclideanDistance2D(point.head(2), obs->GetOrigin().head(2))
            > obs->GetRadius()) return false;
    if(PointInPolygon(point, obs->GetShape())) return true;
    return false;
}
