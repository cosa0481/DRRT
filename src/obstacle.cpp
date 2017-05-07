#include <DRRT/obstacle.h>
#include <DRRT/drrt.h>

using namespace std;

bool timingobs = false;

void Obstacle::ReadObstaclesFromFile(string obstacle_file,
                                     shared_ptr<ConfigSpace>& C)
{
    // Read a polygon from a file
    ifstream read_stream;
    string line, substring;
    stringstream line_stream;
    int num_polygons = 0, num_points;
    read_stream.open(obstacle_file);
    if(read_stream.is_open()) {
        cout << "Obstacle File: " << obstacle_file << endl;
        // Get number of polygons
        getline(read_stream, line);
        num_polygons = stoi(line);
        line = "";
//        cout << "polygons in file: " << num_polygons << endl;
        for(int i = 0; i < num_polygons; i++) {
            // Get number of points in this polygon
            getline(read_stream, line);
            num_points = stoi(line);
            line = "";
//            cout << "number of vertices: " << num_points << endl;
            Eigen::MatrixX2d polygon;
            if(num_points > 0) polygon.resize(num_points,2);
            for(int i = 0; i < num_points; i++) {
                getline(read_stream,line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                polygon.row(i)(0) = stod(substring);
//                cout << "x: " << polygon.row(i)(0) << endl;
                substring = "";
                getline(line_stream, substring);
                polygon.row(i)(1) = stod(substring);
//                cout << "y: " << polygon.row(i)(1) << endl;
                substring = "";
                line = "";
            }
            //getline(read_stream,line);
            //line = "";

            shared_ptr<Obstacle> new_obstacle
                    = make_shared<Obstacle>(3,polygon,C->space_has_theta_);
            new_obstacle->cspace = C;

            // Add to Bullet
            shared_ptr<btCollisionObject> obstacle
                    = make_shared<btCollisionObject>();
            // Set the shape of the obstacle
            shared_ptr<btConvexHullShape> collision_shape
                    = make_shared<btConvexHullShape>();
            for(int i = 0; i < new_obstacle->polygon_.rows(); i++) {
                collision_shape->addPoint(
                            btVector3((btScalar) new_obstacle->polygon_(i,0),
                                      (btScalar) new_obstacle->polygon_(i,1),
                                      (btScalar) -0.1));
                collision_shape->addPoint(
                            btVector3((btScalar) new_obstacle->polygon_(i,0),
                                      (btScalar) new_obstacle->polygon_(i,1),
                                      (btScalar) 0.1));
            }
            obstacle->setCollisionShape(collision_shape.get());
            C->bt_collision_world_->addCollisionObject(obstacle.get());
            // Set origin of the obstacle
//            cout << "center " << i << ": "
//                 << new_obstacle->position_(0) << ","
//                 << new_obstacle->position_(1) << endl;
//            obstacle->getWorldTransform().setOrigin(
//                        btVector3((btScalar) scale*new_obstacle->position_(0),
//                                  (btScalar) scale*new_obstacle->position_(1),
//                                  (btScalar) 0));

            // Add reference to the obstacle's btCollisionObject
            // to Obstacle object
            new_obstacle->collision_object_ = obstacle;
            new_obstacle->collision_shape_ = collision_shape;

            // Add to ConfigSpace
            new_obstacle->AddObsToConfigSpace(C);
        }
    }
    else { cout << "Error opening obstacle file" << endl; }
    read_stream.close();
    cout << "Read in Obstacles: " << num_polygons << endl;
}

void Obstacle::ReadDynamicObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDiscoverableObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDirectionalObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadTimeObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDynamicTimeObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::UpdateObstacles(shared_ptr<ConfigSpace> &C)
{
    lock_guard<mutex> lock(C->cspace_mutex_);
    shared_ptr<ListNode> obstacle_list_node = C->obstacles_->front_;
    shared_ptr<Obstacle> this_obstacle;
    Eigen::VectorXd new_position;
    for(int i = 0; i < C->obstacles_->length_; i++) {
        this_obstacle = obstacle_list_node->obstacle_;
        /// GET NEW OBSTACLE POSITION
        // somehow need to get the transformed object position
        // for each collision object in C->bt_collision_world_:
        //        btTransform updatedWorld;
        //        updatedWorld.setIdentity();
        //        updatedWorld.setOrigin(btVector3(x, y, z));
        //        obstacle->setWorldTransform(updatedWorld);
        new_position = this_obstacle->position_;
        this_obstacle->UpdatePosition(new_position);
        obstacle_list_node = obstacle_list_node->child_;
    }
}

void Obstacle::AddObsToConfigSpace(shared_ptr<ConfigSpace> &C)
{
    lock_guard<mutex> lock(C->cspace_mutex_);
    shared_ptr<Obstacle> this_obstacle = this->GetPointer();
    C->obstacles_->ListPush(this_obstacle);
}

void Obstacle::ChangeObstacleDirection(std::shared_ptr<ConfigSpace> C,
                                       double current_time)
{
    double end_time = C->start_(3);
    // Edges in path ar no longer than this in the time dimension
    double path_time_step = 3.0;

    // Find the path segment of unknown_path that the obstacle is now moving
    // along. Note that future is represented by times closer to S->start_(3)
    // and unknown_path is stored from future (low) to past (high)
    // Nonintuitive I know...
    Eigen::VectorXd high_point, low_point;
    Eigen::ArrayXd temp;
    while(this->next_direction_change_index_ > 0
          && this->unknown_path_.row(this->next_direction_change_index_)(3)
          > current_time) {
        this->next_direction_change_index_ -= 1;
    }

    // Calculate the start and end points of the segment to place in
    // in path and remember next_direction_change_time
    if(this->unknown_path_.row(this->next_direction_change_index_)(3)
            <= current_time
            && this->next_direction_change_index_
            == this->unknown_path_.rows()) {
        // Obstacle has not started moving yet, so assume that
        // we know the first movement segment of the robot
        int index = this->unknown_path_.rows()-1;

        high_point = this->unknown_path_.row(index);
        low_point = this->unknown_path_.row(index-1);
        this->next_direction_change_time_
                = this->unknown_path_.row(index-1)(3);
    } else if(this->unknown_path_.row(this->next_direction_change_index_)(3)
              > current_time
              && this->next_direction_change_index_ <= 1) {
        // Time has progressed further than this obstacle has a path
        // So assume it remains at the end of its path until lifetime expires
        temp << this->unknown_path_.row(0).head(3), current_time;
        high_point = temp;
        temp(3) = current_time-path_time_step;
        low_point = temp;
        this->next_direction_change_time_ = -INF;
    } else {
        high_point = this->unknown_path_.row(
                    this->next_direction_change_index_+1);
        low_point = this->unknown_path_.row(
                    this->next_direction_change_index_);
        this->next_direction_change_time_
                = this->unknown_path_.row(
                    this->next_direction_change_index_)(3);
    }

    // Calculate path, which is a line parallell to the edge
    // [high_point,low_point] and that goes from current_time to end_time in
    // the time dimension
    // The required time must be calculated based on the obstacle's speed

    double mx = (high_point(0) - low_point(0))/(high_point(3) - low_point(3));
    double my = (high_point(1) - low_point(1))/(high_point(3) - low_point(3));

    Eigen::VectorXd ts = Eigen::VectorXd::Zero(ceil(abs(current_time-end_time)
                                                    / path_time_step)+1);

    if(current_time == end_time) {
        ts(0) = current_time;
    } else {
        double val = current_time;
        int count = 0;
        while( count >= end_time ) {
            ts(count) = val;
            val -= path_time_step;
            count++;
        }
        ts(count) = end_time;
    }

    int length = ts.size();
    for(int i = 0; i < length; i++) {
        this->path_.row(i)(0) = low_point(0) + mx*(ts(length-i+1) - low_point(3));
        this->path_.row(i)(1) = low_point(1) + my*(ts(length-i+1) - low_point(3));
        this->path_.row(i)(3) = ts(length-i+1);
    }
}

/// TO BE CALLED IN PLACE OF ExplicitEdgeCheck2D
bool DetectBulletCollision(shared_ptr<Obstacle> &O,
                           Eigen::VectorXd start_point,
                           Eigen::VectorXd end_point)
{

    bool timingcoll = false;
    chrono::steady_clock::time_point t1, f1;
    chrono::steady_clock::time_point t2, f2;
    double delta;

    f1 = chrono::steady_clock::now();

    /// BULLET COLLISION DETECTION
    t1 = chrono::steady_clock::now();
    btCollisionObject* segment = new btCollisionObject();

    // Get length of line segment
    double x = end_point(0) - start_point(0);
    double y = end_point(1) - start_point(1);
    double length = sqrt((x*x) + (y*y));

    // Create box around segment that is longer than it by 2 robot_radius_
    // and that has width 2 robot_radius_
    btCylinderShape* segment_shape = new btCylinderShape(
                btVector3((btScalar) O->cspace->robot_radius_ + length/2,
                          (btScalar) 0.1,
                          (btScalar) O->cspace->robot_radius_ + length/2));

    // Set the collision shape of the segment collision object
    segment->setCollisionShape(segment_shape);
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tEverythingElse: " << delta << " s" << endl;

    // Add the collision object to the collision world
    t1 = chrono::steady_clock::now();
    O->cspace->bt_collision_world_->addCollisionObject(segment);
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tAddObject: " << delta << " s" << endl;

    // Set origin to be center of the line segement
    segment->getWorldTransform().setOrigin(
                btVector3((btScalar) (start_point(0) + end_point(0))/2,
                          (btScalar) (start_point(1) + end_point(1))/2,
                          (btScalar) 0));
    // Calculate angle between the start and end points to determine the
    // yaw rotation of the collision object
    double angle = atan2(end_point(1) - start_point(1),
                         end_point(0) - start_point(0));
    x = cos(angle);
    y = cos(angle);
    segment->getWorldTransform().setRotation(
                btQuaternion((btScalar) 0,              // roll
                             (btScalar) 0,              // pitch
                             (btScalar) atan2(y,x)));   // yaw

    // Check for collisions
    t1 = chrono::steady_clock::now();
    O->cspace->bt_collision_world_->performDiscreteCollisionDetection();
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tCollisionDetection: " << delta << " s" << endl;

    // Number of manifolds is the number of collisions
    t1 = chrono::steady_clock::now();
    int num_manifolds
            = O->cspace->bt_collision_world_->getDispatcher()
            ->getNumManifolds();
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tGetNumManifolds: " << delta << " s" << endl;

    t1 = chrono::steady_clock::now();
    vector<int> num_contacts(num_manifolds);
    shared_ptr<btPersistentManifold> contact_manifold;
    const btCollisionObject* obA;
    const btCollisionObject* obB;
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tDefineManifoldVars: " << delta << " s" << endl;
    for(int i = 0; i < num_manifolds; i++) {
        t1 = chrono::steady_clock::now();
        contact_manifold
                = make_shared<btPersistentManifold>(*O->cspace->bt_collision_world_->getDispatcher()
                ->getManifoldByIndexInternal(i));
        obA = contact_manifold->getBody0();
        obB = contact_manifold->getBody1();
        contact_manifold->refreshContactPoints(obA->getWorldTransform(),
                                               obB->getWorldTransform());
        num_contacts[i] = contact_manifold->getNumContacts();
        t2 = chrono::steady_clock::now();
        delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
        if(timingobs && timingcoll) cout << "\t\tGetNumContacts: " << delta << " s" << endl;
    }
    // Remove this segment collision object from the collision world
    t1 = chrono::steady_clock::now();
    O->cspace->bt_collision_world_->removeCollisionObject(segment);
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tRemoveObject: " << delta << " s" << endl;

    // If there are any manifolds then there is at least one collision.
    /// NEED TO IGNORE OBSTACLE-OBSTACLE COLLISIONS
    t1 = chrono::steady_clock::now();
    for(int i = 0; i < num_manifolds; i++) {
        if(num_contacts[i] > 0) {
//            cout << "collision" << endl;
//            // For each contact point in that manifold
//            for(int j = 0; j < num_contacts[i]; j++) {
//                // Get the contact information
//                btManifoldPoint& pt = contact_manifold->getContactPoint(j);
//                btVector3 ptA = pt.getPositionWorldOnA();
//                double pt_dist = pt.getDistance();
//                cout << "Segment:" << endl;
//                cout << "\tstart: " << start_point(0)
//                     << "," << start_point(1) << endl;
//                cout << "\tend: " << end_point(0)
//                     << "," << end_point(1) << endl;
//                cout << "\tcontact point: " << ptA.getX()
//                     << "," << ptA.getY() << endl;
//                cout << "\tcontact dist: " << pt_dist << endl;
//            }
//            /// EXIT
//            exit(1);
            t2 = chrono::steady_clock::now();
            delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
            if(timingobs && timingcoll) cout << "\t\tLastLoop: " << delta << " s" << endl;
            f2 = chrono::steady_clock::now();
            delta = chrono::duration_cast<chrono::duration<double> >(f2 - f1).count();
            if(timingobs && timingcoll) cout << "\t\tFuncTime: " << delta << " s" << endl;
            return true;
        }
    }
    t2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    if(timingobs && timingcoll) cout << "\t\tLastLoop: " << delta << " s" << endl;
    f2 = chrono::steady_clock::now();
    delta = chrono::duration_cast<chrono::duration<double> >(f2 - f1).count();
    if(timingobs && timingcoll) cout << "\t\tFuncTime: " << delta << " s" << endl;
//    cout << "no collision" << endl;
    return false;
}

void CheckObstacles(shared_ptr<Queue> Q,
                    shared_ptr<KDTree> Tree,
                    shared_ptr<RobotData> Robot,
                    double ball_constant)
{
    bool reached_goal = false;
    bool added, removed;
    shared_ptr<ListNode> obstacle_node;
    shared_ptr<Obstacle> obstacle;
    double time_elapsed;
    double hyper_ball_rad;
    while(!reached_goal) {

        // Update dynamic obstacle positions
        Obstacle::UpdateObstacles(Q->cspace);

        {
            lock_guard<mutex> lock(Tree->tree_mutex_);
            hyper_ball_rad = min(Q->cspace->saturation_delta_,
                             ball_constant*(
                             pow(log(1+Tree->tree_size_)/(Tree->tree_size_),
                                 1/Q->cspace->num_dimensions_)));
        }
        added = false;
        removed = false;
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            obstacle_node = Q->cspace->obstacles_->front_;
            time_elapsed = Q->cspace->time_elapsed_;
            while(obstacle_node != obstacle_node->child_) {
                obstacle = obstacle_node->obstacle_;

                // Remove obstacles whose lifetime is up
                if(!obstacle->sensible_obstacle_ && obstacle->obstacle_used_
                        && (obstacle->start_time_ + obstacle->life_span_
                            <= time_elapsed)) {
                    // Time reached to remove obstacle
                    RemoveObstacle(Tree,Q,obstacle,Tree->root,hyper_ball_rad,
                                   time_elapsed,Q->cspace->move_goal_);
                    removed = true;
                }

                // Add obstacles whose lifetime has begun
                if(!obstacle->sensible_obstacle_ && !obstacle->obstacle_used_
                        && obstacle->start_time_ <= time_elapsed
                        && time_elapsed <= obstacle->start_time_
                                         + obstacle->life_span_) {
                    // Time reached to add obstacle
                    obstacle->obstacle_used_ = true;
                    AddObstacle(Tree,Q,obstacle,Tree->root);
                    {
                        lock_guard<mutex> lock(Robot->robot_mutex);
                        if(Robot->robot_edge_used
                                && Robot->robot_edge->ExplicitEdgeCheck(obstacle)) {
                            Robot->current_move_invalid = true;
                        }
                    }
                    added = true;
                }
                obstacle_node = obstacle_node->child_;
            }
        } // unlock cspace mutex

        {
            lock_guard<mutex> lock(Q->queuetex);
            {
                lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                {
                    lock_guard<mutex> lock(Tree->tree_mutex_);
                    if(removed) {
                        ReduceInconsistency(Q,Q->cspace->move_goal_,
                                            Q->cspace->robot_radius_,
                                            Tree->root, hyper_ball_rad);
                    }
                    if(added) {
                        PropogateDescendants(Q,Tree,Robot);
                        if(!MarkedOS(Q->cspace->move_goal_)) {
                            VerifyInQueue(Q,Q->cspace->move_goal_);
                        }
                        ReduceInconsistency(Q,Q->cspace->move_goal_,
                                            Q->cspace->robot_radius_,
                                            Tree->root,hyper_ball_rad);
                    }
                } // unlock tree mutex
            } // unlock cspace_mutex_
        } // unlock queuetex

        {
            lock_guard<mutex> lock(Robot->robot_mutex);
            reached_goal = Robot->goal_reached;
        }
    }
}

shared_ptr<JList> FindPointsInConflictWithObstacle(shared_ptr<ConfigSpace> &C,
                                                   shared_ptr<KDTree> Tree,
                                                   shared_ptr<Obstacle> &O,
                                                   shared_ptr<KDTreeNode> &root)
{
    shared_ptr<JList> node_list = make_shared<JList>(true);
    double search_range = 0;

    if(1 <= O->kind_ && O->kind_ <= 5) {
        // 2D obstacle
        if(!C->space_has_time_ && !C->space_has_theta_) {
            // Euclidean space without time
            search_range = C->robot_radius_ + C->saturation_delta_ + O->radius_;
            Tree->KDFindWithinRange(node_list,search_range,O->position_);
        } else if(!C->space_has_time_ && C->space_has_theta_) {
            // Dubin's robot without time [x,y,theta]
            search_range = C->robot_radius_ + O->radius_ + PI; // + S->saturation_delta_
            Eigen::Vector3d obs_center_dubins;
            obs_center_dubins << O->position_(0), O->position_(1), PI;
            Tree->KDFindWithinRange(node_list,search_range,obs_center_dubins);
        } else {
            cout << "Error: This type of obstacle not coded for this space"
                 << endl;
        }
    } else if(6 <= O->kind_ && O->kind_ <= 7) {
        // 2D obstacle with time, find points within range of each point along
        // the time path, accumulating all points that are in any of the
        // bounding hyperspheres
        // [x,y,theta,time]

        double base_search_range = C->robot_radius_ + C->saturation_delta_ + O->radius_;
        double search_range;
        int j;
        Eigen::Vector4d query_pose;
        Eigen::Array4d temp, temp1;
        for(int i = 0; i < O->path_.rows(); i++) {
            // Makey query pose the middle of the edge, and add 1/2 edge length
            // through the C-Space to the base_search_range (overestimate)

            if(O->path_.rows() == 1) j = 1;
            else j = i+1;

            temp = O->path_.row(i);
            temp1 = O->path_.row(j);

            temp = temp + temp1;
            temp = temp/2.0;

            temp1 = O->position_;

            query_pose << temp1 + temp;

            if(C->space_has_theta_) query_pose << query_pose, PI;

            search_range = base_search_range
                    + Tree->distanceFunction(O->path_.row(i),
                                             O->path_.row(j))/2.0;

            if(C->space_has_theta_) search_range += PI;

            if(i == 1)
                Tree->KDFindWithinRange(node_list,search_range,query_pose);
            else
                Tree->KDFindMoreWithinRange(node_list,search_range,query_pose);

            if(j == O->path_.rows()) break;
        }
    } else cout << "This case has not been coded yet." << endl;

    return node_list;
}

void AddObstacle(shared_ptr<KDTree> Tree,
                    shared_ptr<Queue> &Q,
                    shared_ptr<Obstacle> &O,
                    shared_ptr<KDTreeNode> root)
{
//    cout << "AddObstacle" << endl;
    // Find all points in conflict with the obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(Q->cspace,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node;
    shared_ptr<double> key = make_shared<double>(0);
    while(node_list->length_ > 0) {
        Tree->PopFromRangeList(node_list,this_node,key);
        // Check all their edges

        // See if this node's neighbors can be reached

        // Get an iterator for this node's out neighbors
        shared_ptr<RrtNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RrtNodeNeighborIterator>(this_node);

        // Iterate through list
        shared_ptr<JListNode> list_item
                = NextOutNeighbor(this_node_out_neighbors);
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        while(list_item->key_ != -1.0) {
            neighbor_edge = list_item->edge_;
            next_item = NextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->ExplicitEdgeCheck(O))
                // Mark edge to neighbor at INF cost
                list_item->edge_->dist_ = INF;
            list_item = next_item;
        }

        // See if this node's parent can be reached
        if(this_node->rrt_parent_used_
                && this_node->rrt_parent_edge_->ExplicitEdgeCheck(O)) {
            // Remove this_node from it's parent's successor list
            this_node->rrt_parent_edge_->end_node_->successor_list_->JListRemove(
                        this_node->successor_list_item_in_parent_);

            // This node now has no parent
            this_node->rrt_parent_edge_->end_node_ = this_node;
            this_node->rrt_parent_edge_->dist_ = INF;
            this_node->rrt_parent_used_ = false;

            VerifyInOSQueue(Q,this_node);
        }
    }

    // Clean up
    Tree->EmptyRangeList(node_list);
}

void RemoveObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue> &Q,
                    std::shared_ptr<Obstacle> &O,
                    std::shared_ptr<KDTreeNode> root,
                    double hyper_ball_rad, double time_elapsed_,
                    std::shared_ptr<KDTreeNode> &move_goal)
{
//    cout << "RemoveObstacle" << endl;
    bool neighbors_were_blocked, conflicts_with_other_obs;

    // Find all points in conflict with obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(Q->cspace,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<double> key = make_shared<double>(0);
//    cout << "points in conflict: " << node_list->length_ << endl;
    while(node_list->length_ > 0) {
//        cout << "point in conflict: " << endl;
        Tree->PopFromRangeList(node_list,this_node,key);
//        cout << this_node->position_ << endl;
        // Check all of their edges

        // See if this node's out neighbors were blocked by the obstacle

        // Get an iterator for this node's out neighbors
        shared_ptr<RrtNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RrtNodeNeighborIterator>(this_node);
        neighbors_were_blocked = false;

        // Iterate through list
        shared_ptr<JListNode> list_item
                = NextOutNeighbor(this_node_out_neighbors);
        shared_ptr<ListNode> o_list_item;
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        shared_ptr<KDTreeNode> neighbor_node;
        while(list_item->key_ != -1.0) {
            neighbor_edge = list_item->edge_;
            neighbor_node = list_item->edge_->end_node_;
            next_item = NextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->dist_ == INF
                    && neighbor_edge->ExplicitEdgeCheck(O)) {
                // This edge used to be in collision with at least one
                // obstacle (at least the obstacle in question)
                // Need to check if could be in conflict with other obstacles

                o_list_item = Q->cspace->obstacles_->front_;

                conflicts_with_other_obs = false;
                shared_ptr<Obstacle> other_obstacle;
                while(o_list_item != o_list_item->child_) {
                    other_obstacle = o_list_item->obstacle_;
                    if(other_obstacle != O
                            && other_obstacle->obstacle_used_
                            && other_obstacle->start_time_ <= time_elapsed_
                            && time_elapsed_ <= (other_obstacle->start_time_
                                               + other_obstacle->life_span_)) {
                        if(neighbor_edge->ExplicitEdgeCheck(other_obstacle)) {
                            conflicts_with_other_obs = true;
                            break;
                        }
                    }
                    o_list_item = o_list_item->child_;
                }

                if(!conflicts_with_other_obs) {
                    // Reset edge length_ to actual cost
                    list_item->edge_->dist_ = list_item->edge_->dist_original_;
                    neighbors_were_blocked = true;
                }
            }
            list_item = next_item;
        }

        if(neighbors_were_blocked) {
            RecalculateLMC(Q,this_node,root,hyper_ball_rad);
            if(this_node->rrt_tree_cost_ != this_node->rrt_LMC_
                    && Q->priority_queue->lessThan(this_node,move_goal))
                VerifyInQueue(Q,this_node);
        }
    }
    Tree->EmptyRangeList(node_list);
    O->obstacle_used_ = false;
}

bool ExplicitEdgeCheck2D(shared_ptr<Obstacle> &O,
                         Eigen::VectorXd start_point,
                         Eigen::VectorXd end_point,
                         double radius)
{
    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;

    // Do a quick check to see if any points on the obstacle might be closer
    // to the edge than robot radius
    if(1 <= O->kind_ && O->kind_ <= 5) {

        // Calculate distance squared from center of the obstacle to the edge
        // projected into the first two dimensions
        double dist_sqrd = DistanceSqrdPointToSegment(O->position_,
                                                      start_point.head(2),
                                                      end_point.head(2));

//        cout << "obs:\n" << O->position_.head(2) << endl;
//        cout << "line:\n" << start_point.head(2) << endl << "--" << endl
//             << end_point.head(2) << endl;
//        cout << "dist: " << sqrt(dist_sqrd) << endl;
//        cout << "min: " << radius+O->radius_ << endl;
        if(dist_sqrd > pow((radius + O->radius_),2)) return false;
    }

    if(O->kind_ == 1) return true; // ball is the obstacle, so in collision
    else if( O->kind_ == 2) return false;
    else if(O->kind_ == 3 || O->kind_ == 5) {
        // Need to check vs all edges in the polygon
        if(O->polygon_.rows() < 2) return false;

        // Start with the last point vs the first point
        Eigen::Vector2d A = O->polygon_.row(O->polygon_.rows()-1);
        Eigen::Vector2d B;
        double seg_dist_sqrd;
        for(int i = 0; i < O->polygon_.rows(); i++) {
            B = O->polygon_.row(i);
            seg_dist_sqrd = SegmentDistSqrd(start_point,end_point,A,B);
//            cout << "dist between edge and polygon edge: " << sqrt(seg_dist_sqrd) << endl;
            if(seg_dist_sqrd < pow(radius,2)) {
                // There is a collision with the 2d projection of the obstacle
//                cout << "p_edge -- traj_edge: " << sqrt(seg_dist_sqrd)
//                     << "\n<\nradius:" << radius << endl;
                if(O->kind_ == 5);
                else return true;
            }
            A = B;
        }
    } else if(O->kind_ == 6 || O->kind_ == 7) {
        // Must check all edges of obstacle path that
        // overlap with robot edge in time

        Eigen::VectorXd early_point, late_point;

        // Make life easier by always checking past to future
        if(start_point(2) < end_point(2)) {
            early_point = start_point;
            late_point = end_point;
        } else {
            early_point = end_point;
            late_point = start_point;
        }

        int first_obs_ind = max(FindIndexBeforeTime(O->path_,early_point(2)),
                                0);
        int last_obs_ind = min(FindIndexBeforeTime(O->path_,late_point(2)),
                               (int)O->path_.rows()-1);

        if(last_obs_ind <= first_obs_ind) return false; // object does not
                                                        // overlap in time

        int i_start, i_end;
        double x_1, y_1, t_1, x_2, y_2, t_2, m_x1, m_y1,
                m_x2, m_y2, t_c, r_x, r_y, o_x, o_y;
        for(i_start = first_obs_ind; i_start < last_obs_ind; i_start++) {
            i_end = i_start + 1;

            x_1 = early_point(0);   // robot start x
            y_1 = early_point(1);   // robot start y
            t_1 = early_point(2);   // robot start time

            x_2 = O->path_.row(i_start)(0) + O->position_(0); // obs start x
            y_2 = O->path_.row(i_start)(1) + O->position_(1); // obs start y
            t_2 = O->path_.row(i_start)(2);                 // obs start time

            // Calculate intermediate quantities (parametric slopes)
            m_x1 = (late_point(0) - x_1)/(late_point(2)-t_1);
            m_y1 = (late_point(1) - y_1)/(late_point(2)-t_1);
            m_x2 = (O->path_.row(i_end)(0) + O->position_(0) - x_2)
                    / (O->path_.row(i_end)(2)-t_2);
            m_y2 = (O->path_.row(i_end)(1) + O->position_(1) - y_2)
                    / (O->path_.row(i_end)(2)-t_2);

            // Solve for time of closest pass of centers
            t_c = (pow(m_x1,2)*t_1 + m_x2*(m_x2*t_2 + x_1 - x_2)
                   - m_x1*(m_x2*(t_1 + t_2) + x_1 - x_2)
                   + (m_y1 - m_y2) * (m_y1*t_1 - m_y2*t_2 - y_1 + y_2))
                  / (pow(m_x1-m_x2,2) + pow(m_y1-m_y2,2));

            // Now bound t_c by the allowable times of robot and obstacle
            if(t_c < max(t_1,t_2)) t_c = max(t_1,t_2);
            else if( t_c > min(late_point(2),O->path_.row(i_end)(2))) {
                t_c = min(late_point(2),O->path_.row(i_end)(2));
            }

            // Finally see if the distance between the robot and the
            // obstacle at t_c is close enough to cause a conflict
            r_x = m_x1*(t_c - t_1) + x_1;   // robot x at t_c
            r_y = m_y1*(t_c - t_1) + y_1;   // robot y at t_c
            o_x = m_x2*(t_c - t_2) + x_2;   // obstacle x at t_c
            o_y = m_y2*(t_c - t_2) + y_2;   // obstacle y at t_c

            if(pow(r_x - o_x,2) + pow(r_y - o_y,2)
                    < pow(O->radius_ + radius,2)) {
                    // Then there is a collision
                    return true;
            }
        }
    }
    return false;
}

bool ExplicitEdgeCheck(shared_ptr<ConfigSpace> &C,
                       shared_ptr<Edge> &edge)
{
    // If ignoring obstacles
    if( C->in_warmup_time_ ) return false;

    chrono::steady_clock::time_point t1,f1;
    chrono::steady_clock::time_point t2,f2;
    double delta;
    if(timingobs) cout << "EXPLICITEDGECHECK" << endl;

    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(C->cspace_mutex_);
        obstacle_list_node = C->obstacles_->front_;
        length = C->obstacles_->length_;
        f1 = chrono::steady_clock::now();
        for( int i = 0; i < length; i++ ) {
            t1 = chrono::steady_clock::now();
            if( edge->ExplicitEdgeCheck(obstacle_list_node->obstacle_) ) {
                t2 = chrono::steady_clock::now();
                delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
                if(timingobs) cout << "ExplicitEdgeCheck(obstacle): " << delta << " s" << endl;
                delta = chrono::duration_cast<chrono::duration<double> >(t2 - f1).count();
                if(timingobs) cout << "ExplicitEdgeCheck: " << delta << " s" << endl;
                C->AddVizEdge(edge,"coll");
                return true;
            }
            t2 = chrono::steady_clock::now();
            delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
            if(timingobs) cout << "ExplicitEdgeCheck(obstacle): " << delta << " s" << endl;
            C->AddVizEdge(edge,"traj");
            obstacle_list_node = obstacle_list_node->child_; // iterate
        }
        f2 = chrono::steady_clock::now();
        delta = chrono::duration_cast<chrono::duration<double> >(f2 - f1).count();
        if(timingobs) cout << "ExplicitEdgeCheck: " << delta << " s" << endl;
    }

    return false;
}

bool QuickCheck2D(shared_ptr<ConfigSpace> &C,
                  Eigen::VectorXd point,
                  shared_ptr<Obstacle> &O)
{
    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;
    if((1 <= O->kind_ && O->kind_ <= 5)
            && C->distanceFunction(O->position_,point) > O->radius_)
        return false;

    if(O->kind_ == 1) return true;
    else if(O->kind_ == 2 || O->kind_ == 4) return false;
    else if(O->kind_ == 3) {
        if(PointInPolygon(point,O->polygon_)) return true;
    } else if(O->kind_ == 5) return false;
    else if(O->kind_ == 6 || O->kind_ == 7) {
        // First transform position_ and polygon to appropriate position_
        // at the time of the point based on the obstacles path through time

        if(point.size() < 3)
            cout << "error: point does not contain time" << endl;

        Eigen::Vector2d offset = FindTransformObjToTimeOfPoint(O,point);

        // Do a quick check based on lower bound of distance to obstacle
        if(C->distanceFunction(O->position_ + offset,point) > O->radius_)
            return false;

        // Transform polygon and do a normal check vs it
        Eigen::ArrayXd polygon;
        polygon = O->original_polygon_.col(0);
        O->polygon_.col(0) = polygon + offset(0);
        polygon = O->original_polygon_.col(1);
        O->polygon_.col(1) = polygon + offset(1);
        if(PointInPolygon(point,O->polygon_)) return true;
    }
    return false;
}

bool QuickCheck(shared_ptr<ConfigSpace> &C, Eigen::VectorXd point)
{
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(C->cspace_mutex_);
        obstacle_list_node = C->obstacles_->front_;
        length = C->obstacles_->length_;

        for(int i = 0; i < length; i++) {
            if(QuickCheck2D(C,point,obstacle_list_node->obstacle_)) return true;
            obstacle_list_node = obstacle_list_node->child_;
        }
    }
    return false;
}

bool ExplicitPointCheck2D(shared_ptr<ConfigSpace> &C,
                          shared_ptr<Obstacle> &O,
                          Eigen::VectorXd point,
                          double radius)
{
//    cout << "ExplicitPointCheck2D" << endl;
    double this_distance = INF;
    double min_distance = C->collision_distance_;

    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;

    if( 1 <= O->kind_ && O->kind_ <= 5 ) {
        // Do a quick check to see if any points on the obstacle
        // might be closer to point than minDist based on the ball
        // around the obstacle

        O->position_(2) = point(2);
//        cout << "Point:\n" << point << endl;
//        if(point(0) > 3
//           && point(0) < 10
//           && point(1) > 3
//           && point(1) < 10) {
//            cout << "Object:\n" << O->position_ << endl;
//            cout << "dist(O,point): " << C->distanceFunction(O->position_,point) << endl;
//            cout << "collision dist: " << C->distanceFunction(O->position_,point) - radius - O->radius_ << endl;
//            cout << "minimum: " << min_distance << endl;
//            cout << "dist > min?: " << ((C->distanceFunction(O->position_,point) - radius - O->radius_) > min_distance) << endl;
//        }

        // Calculate distance from robot boundary to obstacle center
        this_distance = C->distanceFunction(O->position_,point) - radius;
        if(this_distance - O->radius_ > min_distance) return false;
    }

    if(O->kind_ == 1) {
        // Ball is actual obstacle, os we have a new minimum
        this_distance = this_distance - O->radius_;
        if(this_distance < 0.0) return true;
    } else if(O->kind_ == 2) return false;
    else if(O->kind_ == 3) {
//        cout << "PointInPolygon" << endl;
        if(PointInPolygon(point.head(2),O->polygon_)) return true;
//        cout << "the above should be true but it's false" << endl;
        this_distance = sqrt(DistToPolygonSqrd(point,O->polygon_)) - radius;
        if(this_distance < 0.0) return true;
    } else if(O->kind_ == 5) return false;
    else if(O->kind_ == 6 || O->kind_ == 7) {
        // First transform position_ and polygon to appropriate ponsition
        // at the time of the point, based on the obstacle's path through time
        Eigen::Vector2d offset = FindTransformObjToTimeOfPoint(O,point);

        // Do a quick check to see if any points on the obstacle might
        // be closer to the point than min_dist based on the ball around
        // the obstacle

        // Calculate distance from robot boundary to obstacle center
        Eigen::VectorXd now_pos = O->position_;
        now_pos(0) = now_pos(0) + offset(0);
        now_pos(1) = now_pos(1) + offset(1);
        double this_distance = C->distanceFunction(now_pos, point) - radius;
        if(this_distance - O->radius_ > min_distance) return false;

        // Transform polygon and then do the rest of a normal check
        Eigen::ArrayXd polygon = O->original_polygon_.col(0);
        O->polygon_.col(0) = polygon + offset(0);
        polygon = O->original_polygon_.col(1);
        O->polygon_.col(1) = polygon + offset(1);

        if(PointInPolygon(point,O->polygon_)) return true;
        this_distance = sqrt(DistToPolygonSqrd(point,O->polygon_)) - radius;
        if(this_distance < 0.0) return true;
    }

    return false;
}

bool ExplicitPointCheck(shared_ptr<Queue>& Q, Eigen::VectorXd point)
{
    // If ignoring obstacles
    if(Q->cspace->in_warmup_time_) return false;

    // First do quick check to see if the point can be determined in collision
    // with minimal work (quick check is not implicit check)
    if(QuickCheck(Q->cspace,point)) return true;

    // Point is not inside any obstacles but still may be in collision
    // because of the robot radius
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
        obstacle_list_node = Q->cspace->obstacles_->front_;
        length = Q->cspace->obstacles_->length_;

        for(int i = 0; i < length; i++) {
            if(ExplicitPointCheck2D(Q->cspace,obstacle_list_node->obstacle_,
                                    point, Q->cspace->robot_radius_)) return true;
            obstacle_list_node = obstacle_list_node->child_;
        }
    }
    return false;
}

bool ExplicitNodeCheck(shared_ptr<Queue>& Q, shared_ptr<KDTreeNode> node)
{
    return ExplicitPointCheck(Q,node->position_);
}

bool LineCheck(std::shared_ptr<ConfigSpace> C,
               std::shared_ptr<KDTree> Tree,
               std::shared_ptr<KDTreeNode> node1,
               std::shared_ptr<KDTreeNode> node2) {
    // Save the actual angles of these nodes
    double saved_theta1 = node1->position_(2);
    double saved_theta2 = node2->position_(2);
    // Calculate the angle between the two nodes
    double theta = std::atan2(node2->position_(1)-node1->position_(1),
                              node2->position_(0)-node1->position_(0));
    // Make these nodes point in the same direction
    node1->position_(2) = theta;
    node2->position_(2) = theta;
    // Find the trajectory between them (should be a straight line)
    std::shared_ptr<Edge> edge = Edge::NewEdge(C,Tree,node1,node2);
    // Create trajectory straight line from node1 --> node2
    double traj_length = 50;
    Eigen::VectorXd x_traj = Eigen::VectorXd::Zero(traj_length+1);
    Eigen::VectorXd y_traj = Eigen::VectorXd::Zero(traj_length+1);
    double x_val = node1->position_(0);
    double y_val = node1->position_(1);
    double x_dist = abs(node2->position_(0) - x_val);
    double y_dist = abs(node2->position_(1) - y_val);
    int i = 0;
    while(i < traj_length) {
        x_traj(i) = x_val;
        if(node2->position_(0) > node1->position_(0))
            x_val += x_dist/traj_length;
        else x_val -= x_dist/traj_length;

        y_traj(i) = y_val;
        if(node2->position_(1) > node1->position_(1))
            y_val += y_dist/traj_length;
        else y_val -= y_dist/traj_length;

        i++;
    }
    x_traj(i) = x_val;
    y_traj(i) = y_val;
    edge->trajectory_.resize(traj_length+1,2);
    edge->trajectory_.col(0) = x_traj;
    edge->trajectory_.col(1) = y_traj;
    // Check if this straight line trajectory is valid
    bool unsafe = ExplicitEdgeCheck(C,edge);
    // Restore the nodes' original angles (for some reason if I get rid of
    // this it dies but this doesn't mean anything since theta* doesn't use
    // the theta dimension)
    node1->position_(2) = saved_theta1;
    node2->position_(2) = saved_theta2;
    // Return true if there this edge is unsafe and there is no line of sight
    return unsafe;
}
