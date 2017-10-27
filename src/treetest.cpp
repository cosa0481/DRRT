#include <DRRT/rrt.h>

using namespace std;

void TreeTest(CSpace_ptr cspace)
{
    bool random = true;

    double initial_dist, hyper_ball_rad = cspace->saturation_delta_;

    double old_lmc = INF;

    // Sample the CSpace
    Kdnode_ptr new_node;
    Kdnode_ptr closest_node;
    double closest_dist;
    Eigen::MatrixX2d sample_region;
    random = true;
    sample_region.resize(cspace->initial_region_.GetRegion().rows(),
                         Eigen::NoChange_t());
    sample_region = cspace->initial_region_.GetRegion();

    new_node = RandomNodeOrFromStack(cspace);

    cout << "Sampled" << endl;

    if(new_node->InTree()) cout << "new_node in tree" << endl;
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    cout << "Found nearest" << endl;

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    cout << "Saturated" << endl;

    if(NodeCheck(cspace, new_node)) cout << "Node Check true" << endl;


    cspace->kdtree_->Print(cspace->kdtree_->root_);

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    // Extend the graph
    Extend(cspace, new_node, closest_node, hyper_ball_rad);
    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    cout << "Extend complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    // Make graph consistent
    ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
    if(cspace->move_goal_->GetLmc() != old_lmc)
        old_lmc = cspace->move_goal_->GetLmc();

    cout << "ReduceInconsistency complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    ///////////////////////////////////////////////////////////////////////
    /// ITERATION 2
    ///

    new_node = RandomNodeOrFromStack(cspace);

    cout << "Sampled" << endl;

    if(new_node->InTree()) cout << "new_node in tree" << endl;
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    cout << "Found nearest" << endl;

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    cout << "Saturated" << endl;

    if(NodeCheck(cspace, new_node)) cout << "Node Check true" << endl;


    cspace->kdtree_->Print(cspace->kdtree_->root_);

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    // Extend the graph
    Extend(cspace, new_node, closest_node, hyper_ball_rad);
    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    cout << "Extend complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    // Make graph consistent
    ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
    if(cspace->move_goal_->GetLmc() != old_lmc)
        old_lmc = cspace->move_goal_->GetLmc();

    cout << "ReduceInconsistency complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    ///////////////////////////////////////////////////////////////////////
    /// ITERATION 3
    ///

    new_node = RandomNodeOrFromStack(cspace);

    cout << "Sampled" << endl;

    if(new_node->InTree()) cout << "new_node in tree" << endl;
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    cout << "Found nearest" << endl;

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    cout << "Saturated" << endl;

    if(NodeCheck(cspace, new_node)) cout << "Node Check true" << endl;


    cspace->kdtree_->Print(cspace->kdtree_->root_);

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    // Extend the graph
    Extend(cspace, new_node, closest_node, hyper_ball_rad);
    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    cout << "Extend complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    // Make graph consistent
    ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
    if(cspace->move_goal_->GetLmc() != old_lmc)
        old_lmc = cspace->move_goal_->GetLmc();

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;
    cout << "ReduceInconsistency complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    ///////////////////////////////////////////////////////////////////////
    /// ITERATION 4
    ///

    new_node = RandomNodeOrFromStack(cspace);

    cout << "Sampled" << endl;

    if(new_node->InTree()) cout << "new_node in tree" << endl;
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    cout << "Found nearest" << endl;
    cout << "closest_node:\n" << closest_node->GetPosition() << endl;

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    cout << "Saturated" << endl;

    if(NodeCheck(cspace, new_node)) cout << "Node Check true" << endl;


    cspace->kdtree_->Print(cspace->kdtree_->root_);

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    // Extend the graph
    Extend(cspace, new_node, closest_node, hyper_ball_rad);
    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    cout << "Extend complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    // Make graph consistent
    ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
    if(cspace->move_goal_->GetLmc() != old_lmc)
        old_lmc = cspace->move_goal_->GetLmc();

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;
    cout << "ReduceInconsistency complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    ///////////////////////////////////////////////////////////////////////
    /// ITERATION 5
    ///

    new_node = RandomNodeOrFromStack(cspace);

    cout << "Sampled" << endl;

    if(new_node->InTree()) cout << "new_node in tree" << endl;
    {
        lockguard lock(cspace->kdtree_->mutex_);
        closest_dist
                = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
    }

    cout << "Found nearest" << endl;
    cout << "closest_node:\n" << closest_node->GetPosition() << endl;

    // Saturate this node
    initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
    if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
        new_node->SetPosition(
                    new_node->Saturate(new_node->GetPosition(),
                                       closest_node->GetPosition(),
                                       cspace->saturation_delta_,
                                       initial_dist));
    }

    cout << "Saturated" << endl;

    if(NodeCheck(cspace, new_node)) cout << "Node Check true" << endl;


    cspace->kdtree_->Print(cspace->kdtree_->root_);

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    // Extend the graph
    Extend(cspace, new_node, closest_node, hyper_ball_rad);
    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;

    cout << "Extend complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);

    // Make graph consistent
    ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
    if(cspace->move_goal_->GetLmc() != old_lmc)
        old_lmc = cspace->move_goal_->GetLmc();

    cout << "new_node->LChildExist(): " << new_node->LChildExist() << endl;
    cout << "new_node->RChildExist(): " << new_node->RChildExist() << endl;
    cout << "ReduceInconsistency complete" << endl;
    cspace->kdtree_->Print(cspace->kdtree_->root_);
}

