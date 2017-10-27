#include <DRRT/visualizer.h>
#include "../src/list.cpp"

using namespace std;

void Visualizer(CSpace_ptr cspace)
{
    // Build display
    int res_x = 800, res_y = 600, ticks = 50;
    double spacing = 1.0;
    double dist = 36;
    double fov = 420;

    pangolin::CreateWindowAndBind("RRTx", res_x, res_y);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    // Scene graph
    SceneGraph::GLSceneGraph graph;
    // Grid
    SceneGraph::GLGrid grid(ticks, spacing, true);
    graph.AddChild(&grid);
    // Axis
    SceneGraph::GLAxis graph_axis;
    graph_axis.SetPose(0,0,0, 0,0,0);
    graph.AddChild(&graph_axis);
    // Camera render object
    pangolin::OpenGlRenderState stacks_3d(
                pangolin::ProjectionMatrix(res_x, res_y, fov, fov, res_x/2, res_y/2, 0.1, 1000),
                pangolin::ModelViewLookAt(-0.5, 0, -dist, 0,0,0, pangolin::AxisNegZ)
                );
    // View
    pangolin::View view_3d;
    view_3d.SetBounds(0.0, 1.0, 0.0, 1.0, (float)res_x/(float)res_y)
            .SetHandler(new SceneGraph::HandlerSceneGraph(graph, stacks_3d, pangolin::AxisNegZ))
            .SetDrawFunction(SceneGraph::ActivateDrawFunctor(graph, stacks_3d));
    pangolin::DisplayBase().AddDisplay(view_3d);

    // Information
    SceneGraph::GLText *node_count = new SceneGraph::GLText("", 5, -5, 0);
    node_count->set_color(SceneGraph::GLColor(Eigen::Vector4d(0.5, 1.0, 0.5, 1.0)));
    graph.AddChild(node_count);

    // Display parameters
    KdnodeList nodes;
    KdnodeListNode_ptr node_item = make_shared<KdnodeListNode>();
    KdnodeListNode_ptr transfer_item = make_shared<KdnodeListNode>();
    double node_scale = 0.15;
    int node_color_index = 4;
    vector<Eigen::VectorXd> poses;
    double obs_num = 0;
    vector<int> current_path_idxs;
    vector<SceneGraph::GLLineStrip*> obstacles;
    vector<Eigen::VectorXd> obs_times;
    vector<Eigen::MatrixXd> obs_paths;
    Region obs_region;
    Eigen::VectorXd obs_origin;
    EdgeList colls;
    EdgeListNode_ptr transfer_edge = make_shared<EdgeListNode>();
    EdgeListNode_ptr edge_item = make_shared<EdgeListNode>();

    //this_thread::sleep_for(chrono::milliseconds(1000));

    // Add obstacles
    {
        lockguard lock(cspace->mutex_);
        obstacles = vector<SceneGraph::GLLineStrip*>(cspace->obstacles_->GetLength());
        obs_times = vector<Eigen::VectorXd>(cspace->obstacles_->GetLength());
        obs_paths = vector<Eigen::MatrixXd>(cspace->obstacles_->GetLength());
        current_path_idxs = vector<int>(cspace->obstacles_->GetLength());

        ObstacleListNode_ptr obs_item = cspace->obstacles_->GetFront();
        Obstacle_ptr obstacle;
        while(!obs_item->IsEmpty()) {
            obs_item->GetData(obstacle);
            obs_region = obstacle->GetShape();
            obs_origin = obstacle->GetOrigin();
            SceneGraph::GLLineStrip *gl_obs = new SceneGraph::GLLineStrip();
            for(int i = 0; i < obs_region.GetRegion().rows(); i++) {
                gl_obs->SetPoint(obs_region.GetRegion().row(i)(1),
                                 obs_region.GetRegion().row(i)(0),
                                 0.0);
            }
            gl_obs->SetPoint(obs_region.GetRegion().row(0)(1),
                             obs_region.GetRegion().row(0)(0),
                             0.0);

            gl_obs->SetColor(SceneGraph::GLColor(0,0,0));
            gl_obs->SetPose(obs_origin(1), obs_origin(0), 0.0, 0.0, 0.0, 0.0);
            obstacles[obs_num] = gl_obs;
            if(!cspace->static_obstacles_) {
                obs_times[obs_num] = obstacle->GetPathTimes();
                obs_paths[obs_num] = obstacle->GetPath();
                current_path_idxs[obs_num++] = 0;
            }
            graph.AddChild(gl_obs);

            obs_item = obs_item->GetChild();
        }
    }

    // Add an indicator of the goal state
    SceneGraph::GLBox *goal_box = new SceneGraph::GLBox();
    SceneGraph::GLAxis *goal_axis = new SceneGraph::GLAxis();
    goal_box->SetScale(0.5);
    goal_box->SetCheckerboard(1);
    goal_axis->AddChild(goal_box);
    goal_axis->SetPose(cspace->start_(1), cspace->start_(0), 0.0, 0.0, 0.0, PI/2 - cspace->start_(2));
    graph.AddChild(goal_axis);

    int num_nodes = 0;
    bool goal_reached = false;
    double now;
    while(!pangolin::ShouldQuit())
    {
        // Clear screen
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        // Get items into list from cspace
        {
            lockguard lock(cspace->mutex_);
            now = cspace->elapsed_time_;
            for(int i = 0; i < cspace->collisions_->GetLength(); i++) {
                cspace->collisions_->Pop(transfer_edge);
                colls.Push(transfer_edge);
            }
            colls.Pop(edge_item);
            for(int i = 0; i < cspace->new_nodes_->GetLength(); i++) {
                cspace->new_nodes_->Pop(transfer_item);
                nodes.Push(transfer_item);
            }
            nodes.Pop(node_item);
        }
        {
            lockguard lock(cspace->robot_->mutex);
            poses = cspace->robot_->poses;
            cspace->robot_->poses.clear();  // Clear container in cspace
        }

        // Create SceneGraph objects for nodes and poses
        // K-D nodes
        while(!node_item->IsEmpty()) {
            SceneGraph::GLBox *box = new SceneGraph::GLBox();
            box->SetScale(node_scale);
            box->SetCheckerboard(node_color_index);
            SceneGraph::GLAxis *axis = new SceneGraph::GLAxis();
            axis->SetScale(node_scale);
            axis->AddChild(box);

            Kdnode_ptr kdnode = make_shared<Kdnode>();
            node_item->GetData(kdnode);

            axis->SetPose(kdnode->GetPosition()(1), kdnode->GetPosition()(0), 0.0,
                          0.0, 0.0, PI/2 - kdnode->GetPosition()(2));
            graph.AddChild(axis);

            num_nodes++;
            stringstream num_nodes_stream("");
            num_nodes_stream << "Total Nodes: " << num_nodes + 0;
            node_count->SetText(num_nodes_stream.str());

            nodes.Pop(node_item);
        }

        // Poses
        for(int i = 0; i < poses.size(); i++) {
            SceneGraph::GLAxis *pose = new SceneGraph::GLAxis();
            pose->SetPose(poses.at(i)(1), poses.at(i)(0), 0.0,
                          0.0, 0.0, PI/2 - poses.at(i)(2));
            graph.AddChild(pose);
        }

        // Obstacles
        if(!cspace->static_obstacles_) {
            for(int i = 0; i < obstacles.size(); i++) {
                if(current_path_idxs[i] < obs_times[i].size()
                        && obs_times[i](current_path_idxs[i]) < now) {
                    obstacles[i]->SetPose(obs_paths[i].row(current_path_idxs[i])(1),
                                          obs_paths[i].row(current_path_idxs[i])(0),
                                          0.0,
                                          0.0, 0.0, 0.0);
                    current_path_idxs[i] += 1;
                }
            }
        }

        // Collisions
        if(DEBUGBULLET) {
            while(!edge_item->IsEmpty()) {
                SceneGraph::GLLineStrip *col_edge = new SceneGraph::GLLineStrip();
                Edge_ptr dedge = Edge::NewEdge();
                edge_item->GetData(dedge);
                col_edge->SetPoint(dedge->GetStart()->GetPosition()(1),
                                   dedge->GetStart()->GetPosition()(0),
                                   0.0);
                col_edge->SetPoint(dedge->GetEnd()->GetPosition()(1),
                                   dedge->GetEnd()->GetPosition()(0),
                                   0.0);

                // Show in red
                col_edge->SetColor(SceneGraph::GLColor(Eigen::Vector4d(1,0,0,1)));
                graph.AddChild(col_edge);

                colls.Pop(edge_item);
            }
        }

        {
            lockguard lock(cspace->robot_->mutex);
            goal_reached = cspace->robot_->goal_reached;
        }

        if(goal_reached) {
            break;
        }

        pangolin::FinishFrame();
    }
}
