#include <DRRT/collision_detection.h>
#include <DRRT/data_structures.h>

KdnodeList_ptr FindPointsInConflictWithObstacle(KdTree_ptr tree, Obstacle_ptr obs)
{
    KdnodeList_ptr node_list;
    if(NUM_DIM == 3) {
        Eigen::Vector3d obs_center(obs->GetOrigin()(0), obs->GetOrigin()(1), PI);
        node_list = tree->FindWithinRange(obs->cspace->robot_radius_, obs_center);
    } else {
        std::cout << "Error: FindPointsInConflictWithObstacle not implemented." << std::endl;
        node_list = std::make_shared<KdnodeList>();
    }
    return node_list;
}

bool DetectCollision(Obstacle_ptr &obs, Edge_ptr edge)
{

}

bool LineCheck(Kdnode_ptr node1, Kdnode_ptr node2)
{

}

bool NodeCheck(std::shared_ptr<ConfigSpace> cspace, Kdnode_ptr node)
{

}

bool PointCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point)
{

}

bool PointCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point)
{

}

bool EdgeCheck(Obstacle_ptr obstacle, Edge_ptr edge)
{

}

bool EdgeCheck(std::shared_ptr<ConfigSpace> cspace, Edge_ptr edge)
{

}

bool EdgeCheck2D(std::shared_ptr<ConfigSpace> cspace, Edge_ptr edge)
{

}

bool QuickCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point)
{

}

bool QuickCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point)
{

}
