#include <DRRT/drrt.h>
#include <DRRT/visualizer.h>

using namespace std;

/// VISUALIZER FUNCTION
void visualizer(shared_ptr<KDTree> Tree,
                shared_ptr<RobotData> Robot,
                shared_ptr<Queue> Q)
{
    int resX = 800, resY = 600, ticks = 100;
    double spacing = 1.0; // straighttest:1.0 smalltest:1.0 largetest:5.0
    double dist = 10; // straighttest:10 smalltest:10 largetest:20
    double fov = 420; // field of view (this just worked best)

    bool show_collision_edges = false;
    bool show_trajectories = false;
    bool draw_current_traj = false;

    /// Build display
    // Create OpenGL window
    pangolin::CreateWindowAndBind("RRTx",resX,resY);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    // Define grid object
    SceneGraph::GLGrid glGrid(ticks,spacing,true);
    glGraph.AddChild(&glGrid);

    // Define axis object
    SceneGraph::GLAxis glAxis;
    glAxis.SetPose(0,0,0,0,0,0);
    glGraph.AddChild(&glAxis);

    // Define camera render object
    pangolin::OpenGlRenderState stacks3d(
                pangolin::ProjectionMatrix(resX,resY,fov,fov,resX/2,resY/2,
                                           0.1,1000),
                pangolin::ModelViewLookAt(-dist,-dist,-dist, 0,0,0,
                                          pangolin::AxisNegZ));

    // Define view for base container
    pangolin::View view3d;

    // Set view location on screen and add handler for updating of model view
    // matrix (stacks3d)
    view3d.SetBounds(0.0,1.0,0.0,1.0,(float)resX/(float)resY)
            .SetHandler(new SceneGraph::HandlerSceneGraph(
                            glGraph,stacks3d,pangolin::AxisNegZ))
            .SetDrawFunction(SceneGraph::ActivateDrawFunctor(
                                 glGraph,stacks3d));

    // Add view to base container as child
    pangolin::DisplayBase().AddDisplay(view3d);

    // For the obstacles
    List obstacles;
    vector<Eigen::Vector2d> origins;
    shared_ptr<Obstacle> this_obstacle;
    shared_ptr<Obstacle> prev_obstacle;
    SceneGraph::GLLineStrip* polygon;
    vector<SceneGraph::GLLineStrip*> polygons;
    Eigen::MatrixX2d obstacle_pos;

    {
        lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
        obstacles = *Q->cspace->obstacles_;
    }
    int num_obstacles = obstacles.length_;
    for(int i = 0; i < num_obstacles; i++) {
        obstacles.ListPop(this_obstacle);
        obstacle_pos = this_obstacle->GetPosition();
        polygon = new SceneGraph::GLLineStrip();
        for( int j = 0; j < obstacle_pos.rows(); j++) {
            polygon->SetPoint(Eigen::Vector3d(obstacle_pos.row(j)(1),
                                              obstacle_pos.row(j)(0),
                                              0.0));
        }
        polygon->SetPoint(Eigen::Vector3d(obstacle_pos.row(0)(1),
                                          obstacle_pos.row(0)(0),
                                          0.0));
        // Show obstacles in black
        polygon->SetColor(SceneGraph::GLColor(Eigen::Vector4d(0,0,0,1)));
        glGraph.AddChild(polygon);
    }

    // For the K-D Tree nodes
    vector<shared_ptr<KDTreeNode>> nodes;
    int num_nodes = 0;
    Eigen::VectorXd kdnode;
    SceneGraph::GLText* node_count
            = new SceneGraph::GLText("", 5,-5,0);
    glGraph.AddChild(node_count);

    SceneGraph::GLBox* box;
    SceneGraph::GLAxis* axis;

    // For edges
    SceneGraph::GLLineStrip* traj_line;
    vector<SceneGraph::GLLineStrip*> traj_lines;
    SceneGraph::GLLineStrip* edge;
    vector<shared_ptr<Edge>> collisions_;
    vector<shared_ptr<Edge>> trajectories_;

    // For robot poses
    vector<Eigen::VectorXd> poses;
    SceneGraph::GLAxis* pose;
    Eigen::VectorXd current_pose;

    SceneGraph::GLLineStrip* path;

    // Default hook for exiting: Esc
    // Default hook for fullscreen: tab
    bool path_drawn = false;
    bool moved = false;
    bool move_switch = false;
    while( !pangolin::ShouldQuit() ) {
        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor4f(1.0f,1.0f,1.0f,1.0f);

        // Draw obstacles
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            obstacles = *Q->cspace->obstacles_;
            moved = Q->cspace->obstacles_moved_;
        }
        if(move_switch && !moved) move_switch = false;
        num_obstacles = obstacles.length_;
        if(moved /*&& !move_switch*/) {

            for(int i = 0; i < num_obstacles; i++) {
                obstacles.ListPop(this_obstacle);
                obstacle_pos = this_obstacle->GetPosition();
                polygon = new SceneGraph::GLLineStrip();
                for( int j = 0; j < obstacle_pos.rows(); j++) {
                    polygon->SetPoint(Eigen::Vector3d(obstacle_pos.row(j)(1),
                                                      obstacle_pos.row(j)(0),
                                                      0.0));
                }
                polygon->SetPoint(Eigen::Vector3d(obstacle_pos.row(0)(1),
                                                  obstacle_pos.row(0)(0),
                                                  0.0));
                // Show obstacles in black
                polygon->SetColor(SceneGraph::GLColor(Eigen::Vector4d(0,0,0,1)));
                glGraph.AddChild(polygon);
            }
            move_switch = true;
            path_drawn = false;
        }


        // Draw any-angle best path
        if(!path_drawn) {
            path = new SceneGraph::GLLineStrip();
            path->SetReference(0,0,0);
            {
                lock_guard<mutex> lock(Robot->robot_mutex);
                for( int i = 0; i < Robot->best_any_angle_path.size(); i++) {
                    path->SetPoint(Robot->best_any_angle_path.at(i)(1),
                                Robot->best_any_angle_path.at(i)(0),
                                0);
                }
            }
            glGraph.AddChild(path);
            path_drawn = true;
            if(path->GetPose()[0] == 0) path_drawn = false;
        }

        for(auto line : traj_lines) {
            glGraph.RemoveChild(line);
        }

        // Draw current trajectory
        if(draw_current_traj) {
            {
                lock_guard<mutex> lock(Robot->robot_mutex);
                {
                    lock_guard<mutex> lock(Tree->tree_mutex_);
                    shared_ptr<KDTreeNode> node = make_shared<KDTreeNode>();
                    shared_ptr<double> dist = make_shared<double>(INF);
                    Tree->KDFindNearest(node,dist,Robot->robot_pose);
                    Eigen::MatrixXd traj = node->rrt_parent_edge_->trajectory_;
                    traj_lines = vector<SceneGraph::GLLineStrip*>(traj.rows());
                    while(node->rrt_parent_used_) {
                        traj_line = new SceneGraph::GLLineStrip();
                        traj_line->SetReference(0,0,0);
                        for(int i = 1; i < traj.rows(); i++) {
                            traj_line->SetPoint(traj(i-1,1),
                                                traj(i-1,0),
                                                0);
                            traj_line->SetPoint(traj(i,1),
                                                traj(i,0),
                                                0);
                        }
                        traj_line->SetColor(SceneGraph::GLColor(Eigen::Vector4d(0.5,1,1,1)));
                        traj_lines.push_back(traj_line);
                        glGraph.AddChild(traj_line);
                        node = node->rrt_parent_edge_->end_node_;
                    }
                }
            }
        }

        // Add K-D Tree nodes to the frame
        {
            lock_guard<mutex> lock(Tree->tree_mutex_);
            nodes = Tree->nodes_;
//            cout << "nodes to display: " << nodes.size() << endl;
            /// Write num_nodes to screen with label "Total Nodes: "
            for( int k = 0; k < nodes.size(); k++ ) {
                box = new SceneGraph::GLBox();
                box->SetScale(0.15);
                box->SetCheckerboard(4);
                axis = new SceneGraph::GLAxis();
                axis->SetAxisSize(0.15);
                axis->AddChild(box);

                kdnode = nodes.at(k)->position_;
                axis->SetPose(kdnode(1),kdnode(0),0.0,
                                 0.0,0.0,PI/2 - kdnode(2));
                glGraph.AddChild(axis);
                Tree->RemoveVizNode(nodes.at(k));
                num_nodes++;
                stringstream disp_nodes_stream("");
                disp_nodes_stream << "Total Nodes: " << num_nodes + 0;
                node_count->SetText(disp_nodes_stream.str());
                node_count->set_color(SceneGraph::GLColor(Eigen::Vector4d(0.5,1,0.5,1)));
//                cout << disp_nodes_stream.str() << endl;
            }
        }

        /// Display collision lines
        if(show_trajectories) {
            {
                lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                trajectories_ = Q->cspace->trajectories_;
                for(int p = 0; p < trajectories_.size(); p++) {
                    edge = new SceneGraph::GLLineStrip();
                    edge->SetPoint(Eigen::Vector3d(
                                trajectories_.at(p)->start_node_->position_(1),
                                trajectories_.at(p)->start_node_->position_(0),
                                0.0));
                    edge->SetPoint(Eigen::Vector3d(
                                trajectories_.at(p)->end_node_->position_(1),
                                trajectories_.at(p)->end_node_->position_(0),
                                0.0));
                    Q->cspace->RemoveVizEdge(trajectories_.at(p),"traj");
                    // Show in green
                    edge->SetColor(SceneGraph::GLColor(Eigen::Vector4d(0,1,0,1)));
                    glGraph.AddChild(edge);
                }
            }
        }
        if(show_collision_edges) {
            {
                lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                collisions_ = Q->cspace->collisions_;
                for(int p = 0; p < collisions_.size(); p++) {
                    edge = new SceneGraph::GLLineStrip();
                    edge->SetPoint(Eigen::Vector3d(
                                collisions_.at(p)->start_node_->position_(1),
                                collisions_.at(p)->start_node_->position_(0),
                                0.0));
                    edge->SetPoint(Eigen::Vector3d(
                                collisions_.at(p)->end_node_->position_(1),
                                collisions_.at(p)->end_node_->position_(0),
                                0.0));
                    Q->cspace->RemoveVizEdge(collisions_.at(p),"coll");
                    // Show in red
                    edge->SetColor(SceneGraph::GLColor(Eigen::Vector4d(1,0,0,1)));
                    glGraph.AddChild(edge);
                }
            }
        }


        // Update robot pose (x,y,z, r,p,y)
        {
            lock_guard<mutex> lock(Robot->robot_mutex);
            current_pose = Robot->robot_pose;
        }
        if( poses.size() == 0 || current_pose != poses.back() ) {
            poses.push_back(current_pose);
            pose = new SceneGraph::GLAxis();
            pose->SetPose(poses.back()(1),poses.back()(0),0.0,
                          0.0,0.0,PI/2 - poses.back()(2));
            glGraph.AddChild(pose);
        }

        // Swap frames and process events
        pangolin::FinishFrame();
    }
    delete(box);
    delete(axis);
    delete(pose);
    delete(node_count);
    delete(traj_line);
}
