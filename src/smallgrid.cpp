#include <DRRT/rrt.h>

using namespace std;

void printRrtxPath(Kdnode_ptr &leaf)
{
    std::cout << "\nRRTx Path" << std::endl;
    while(leaf->RrtParentExist()) {
        cout << "node: " << leaf->GetLmc() << "\n" << leaf->GetPosition() << endl;
        cout << "VVVVVVVV" << endl;
        Edge_ptr rrt_parent_edge = Edge::NewEdge();
        leaf->GetRrtParentEdge(rrt_parent_edge);
        leaf = rrt_parent_edge->GetEnd();
    }
    cout << leaf->GetPosition() << endl;
}

void SmallGrid(CSpace_ptr cspace)
{
    Eigen::Vector3d position;
    double closest_dist, initial_dist;
    Kdnode_ptr closest_node = make_shared<Kdnode>();

    Kdnode_ptr new_node = RandomNode(cspace);
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    // Extend
    // Calculate ball radius
    double hyper_ball_rad = min(cspace->saturation_delta_,
                         cspace->ball_constant_*(pow( log(1 + cspace->kdtree_->size_)/cspace->kdtree_->size_,
                                             1/NUM_DIM)));
    cout << "hyper_ball_rad: " << hyper_ball_rad << endl;
    Extend(cspace, new_node, closest_node, hyper_ball_rad);

    ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, hyper_ball_rad);

    new_node = RandomNode(cspace);
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    // Extend
    // Calculate ball radius
    hyper_ball_rad = min(cspace->saturation_delta_,
                         cspace->ball_constant_*(pow( log(1 + cspace->kdtree_->size_)/cspace->kdtree_->size_,
                                             1/NUM_DIM)));
    cout << "hyper_ball_rad: " << hyper_ball_rad << endl;
    Extend(cspace, new_node, closest_node, hyper_ball_rad);

    ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, hyper_ball_rad);


//    position(0) = -5;
//    position(1) = 0;
//    position(2) = -3*PI/4;
//    Kdnode_ptr node_1 = make_shared<Kdnode>(position);
//    closest_dist = cspace->kdtree_->FindNearest(closest_node, node_1->GetPosition());
//    Extend(cspace, node_1, closest_node, 1);
//    ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, cspace->robot_->radius);
//    cout << "Inserted node_1" << endl;

//    cspace->kdtree_->FindNearest(closest_node, cspace->end_);
//    printRrtxPath(closest_node);

//    position(0) = 5;
//    position(1) = 0;
//    position(2) = -3*PI/4;
//    Kdnode_ptr node_2 = make_shared<Kdnode>(position);
//    closest_dist = cspace->kdtree_->FindNearest(closest_node, node_2->GetPosition());
//    Extend(cspace, node_2, closest_node, 1);
//    ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, cspace->robot_->radius);
//    cout << "Inserted node_2" << endl;

//    cspace->kdtree_->FindNearest(closest_node, cspace->end_);
//    printRrtxPath(closest_node);

//    position(0) = -2.5;
//    position(1) = -2.5;
//    position(2) = -3*PI/4;
//    Kdnode_ptr node_3 = make_shared<Kdnode>(position);
//    closest_dist = cspace->kdtree_->FindNearest(closest_node, node_3->GetPosition());
//    Extend(cspace, node_3, closest_node, 1);
//    ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, cspace->robot_->radius);
//    cout << "Inserted node_3" << endl;

    cspace->kdtree_->FindNearest(closest_node, cspace->end_);
    printRrtxPath(closest_node);

    double start_time = GetTimeNs(cspace->start_time_);
    double now = start_time;
    double elapsed_time = 0.0;

    Eigen::Vector3d robot_pose;

    bool reached_goal = false;
    while(!reached_goal)
    {
        now = GetTimeNs(cspace->start_time_);
        elapsed_time = now - start_time;
        {
            lockguard lock(cspace->mutex_);
            cspace->elapsed_time_ = elapsed_time/1000000000;
        }
        {
            lockguard lock(cspace->robot_->mutex);
            robot_pose = cspace->robot_->pose;
            reached_goal = cspace->robot_->goal_reached;
        }
//        Kdnode_ptr closest_node = make_shared<Kdnode>();
//        cspace->kdtree_->FindNearest(closest_node, robot_pose);
//        printRrtxPath(closest_node);
        this_thread::sleep_for(chrono::milliseconds(1000));  // 1 second
    }
}
