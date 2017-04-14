#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <DRRT/distancefunctions.h>

class ConfigSpace;
class JList;
class KDTree;
class KDTreeNode;
class Edge;
struct Queue;
struct RobotData;

class Obstacle : public std::enable_shared_from_this<Obstacle>
{
public:

    int kind_;  // 1 = ball
                // 2 = axis aligned hyperrectangle
                // 3 = polygon
                // 4 = polygon with safe direction
                // 5 = high dimensional prismatic polygon
                // 6 = polygon that moves in time along a predefined path
                // 7 = similar to 6 but robot does not "know" path a priori

    double start_time_; // obstacle appears this long after start of run (0)
    double life_span_;  // lifespan of obstacle (INF)
    bool obstacle_used_; // if true, obstacle will not be checked
    bool sensible_obstacle_;    // true if this obstacle can be sensed by
                                // the robot. set to false after sensed
                                // (false)
    bool obstacle_used_after_sense_;    // obstacle_used_ is set to this value
                                        // after the robot senses this
                                        // obstacle (false)

    Eigen::VectorXd position_;   // initial position of obstacle

    // Data for all D-dimensional ball obstacles (kind=1) as a
    // bound on obstacle (all kinds)
    double radius_;

    // Data for axis aligned D-dimensional hyperrectangle obstacle (kind=2)
    Eigen::Vector2d span_;

    // Data for polygon (kind=3,6)
    Eigen::MatrixX2d polygon_;

    // Direction is used for directional polygon (kind=4)
    char direction_;

    // Used only for high dimensional prismatic polygon (kind=5)
    Eigen::VectorXd prism_span_min_;
    Eigen::VectorXd prism_span_max_;

    // Stuff for time obstacles (kind=6,7)
    double velocity_;       // speed obstacle moves with
    Eigen::MatrixXd path_;  // offset path robot follows (path_(1,:)=0,0)

    Eigen::MatrixX2d original_polygon_;

    // Stuff for time obstacles with "unknown" path (kind=7)
    Eigen::MatrixXd unknown_path_;
    double next_direction_change_time_;
    int next_direction_change_index_;
    double last_direction_change_time_;

    // Constructors
    // Empty Obstacle
    Obstacle(int kind) : kind_(kind) {}

    // Ball
    Obstacle(int kind, Eigen::VectorXd position, double radius)
        : kind_(kind), start_time_(0.0), life_span_(INF),
          obstacle_used_(false), sensible_obstacle_(false),
          obstacle_used_after_sense_(false), position_(position),
          radius_(radius)
    {}

    // Hyperrectangle
    Obstacle(int kind, Eigen::VectorXd position, Eigen::VectorXd span)
        : kind_(kind), start_time_(0.0), life_span_(INF),
          obstacle_used_(false), sensible_obstacle_(false),
          obstacle_used_after_sense_(false), position_(position),
          span_(span)
    {
        double sum = 0;
        for(int i = 0; i < span_.size(); i++) {
            sum += span_(i)*span_(i);
        }
        radius_ = sqrt(sum);
    }

    // Polygon
    Obstacle(int kind, Eigen::MatrixX2d polygon, bool ConfigSpace_has_theta)
        : kind_(kind), start_time_(0.0), life_span_(INF),
          obstacle_used_(false), sensible_obstacle_(false),
          obstacle_used_after_sense_(false), polygon_(polygon)
    {
        Eigen::VectorXd pos;
        if(ConfigSpace_has_theta) {
            pos = Eigen::Vector3d();
            pos(2) = 0.0;
        } else {
            pos = Eigen::Vector2d();
        }
        pos(0) = (polygon.col(0).maxCoeff() + polygon.col(0).minCoeff())/2.0;
        pos(1) = (polygon.col(1).maxCoeff() + polygon.col(1).minCoeff())/2.0;
        position_ = pos;

        double dist, max = 0;
        for(int i = 0; i < polygon.rows(); i++) {
            dist = sqrt(pow(polygon.row(i)(0) - position_(0),2)
                        + pow(polygon.row(i)(1) - position_(1),2));
            if(dist > max) max = dist;
        }

        radius_ = max;  // distance to furthest point

        span_ = Eigen::Vector2d(-1.0,-1.0);
    }

    // Polygon with safe direction
    Obstacle(int kind, Eigen::Matrix2Xd polygon, char direction)
        : kind_(kind), start_time_(0.0), life_span_(INF),
          obstacle_used_(false), sensible_obstacle_(false),
          obstacle_used_after_sense_(false), polygon_(polygon),
          direction_(direction)
    {
        Eigen::Vector2d pos;
        pos(0) = (polygon.col(0).maxCoeff() + polygon.col(0).minCoeff())/2.0;
        pos(1) = (polygon.col(1).maxCoeff() + polygon.col(1).minCoeff())/2.0;
        position_ = pos;

        double dist, max = 0;
        for(int i = 0; i < polygon.rows(); i++) {
            dist = sqrt(pow(polygon.row(i)(0) - position_(0),2)
                        + pow(polygon.row(i)(1) - position_(1),2));
            if(dist > max) max = dist;
        }

        radius_ = max;  // distance to furthest point

        span_ = Eigen::Vector2d(-1.0,-1.0);
    }

    // Read in obstacles from files
    static void ReadObstaclesFromFile(std::string obstacle_file,
                                      std::shared_ptr<ConfigSpace> &C);
    static void ReadDynamicObstaclesFromFile(std::string obstacle_file);
    static void ReadDiscoverableObstaclesFromFile(std::string obstacle_file);
    static void ReadDirectionalObstaclesFromFile(std::string obstacle_file);
    static void ReadTimeObstaclesFromFile(std::string obstacle_file);
    static void ReadDynamicTimeObstaclesFromFile(std::string obstacle_file);

    // Moves obstacles around or remove them
    static void UpdateObstacles(std::shared_ptr<ConfigSpace>& C);
    // Adds the obstacle to the ConfigSpace
    void AddObsToConfigSpace(std::shared_ptr<ConfigSpace>& C);
    // Decrease life of obstacle
    void DecreaseLife() { this->life_span_ -= 1.0; }
    // Get shared_ptr to this Obstacle
    std::shared_ptr<Obstacle> GetPointer() { return shared_from_this(); }

    // For obstacle type 7 (time obstacles with unknown paths to the robot)
    // this is used to calculate the current path (for collision checking)
    // from unknown_path (which describes how the obstacle moves vs time)
    // based on current time. Note that times in the future are closer
    // to S->start_(3) for [x,y,theta,time]
    void ChangeObstacleDirection(std::shared_ptr<ConfigSpace> C,
                                 double current_time);

};

// Obstacle thread function for adding and removing obstacles
void CheckObstacles(std::shared_ptr<Queue> Q,
                    std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<RobotData> Robot,
                    double ball_constant);

// This returns a -rangeList- (see KDTree code) containing all points
// that are in conflict with the obstacle. Note that rangeList must
// be DESTROYED PROPERLY using L.EmptyRangeList to avoid problems -collision-
std::shared_ptr<JList> FindPointsInConflictWithObstacle(
        std::shared_ptr<ConfigSpace> &C,
        std::shared_ptr<KDTree> Tree,
        std::shared_ptr<Obstacle> &O,
        std::shared_ptr<KDTreeNode> &root);

// This adds the obstacle (checks for edge conflicts with the obstactle
// and then puts the affected nodes into the appropriate heaps -collision-
void AddObstacle(std::shared_ptr<KDTree> Tree,
                 std::shared_ptr<Queue> &Q,
                 std::shared_ptr<Obstacle> &O,
                 std::shared_ptr<KDTreeNode> root);

// This removes the obstacle (checks for edge conflicts with the obstacle
// and then puts the affected nodes into the appropriate heaps)
void RemoveObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue> &Q,
                    std::shared_ptr<Obstacle> &O,
                    std::shared_ptr<KDTreeNode> root,
                    double hyper_ball_rad, double time_elapsed_,
                    std::shared_ptr<KDTreeNode>& move_goal );

// Checks if the edge between the points is in collision with the obstacle
// (the point is closer than robot radius to the edge)
bool ExplicitEdgeCheck2D(std::shared_ptr<Obstacle> &O,
                         Eigen::VectorXd start_point,
                         Eigen::VectorXd end_point,
                         double radius);

// Checks if the edge is in collision with any obstacles in the C-space
// Returns true if there the edge is in collision
bool ExplicitEdgeCheck(std::shared_ptr<ConfigSpace> &C,
                       std::shared_ptr<Edge> &edge);

bool QuickCheck2D(std::shared_ptr<ConfigSpace> &C,
                  Eigen::Vector2d point,
                  std::shared_ptr<Obstacle> &O);

bool QuickCheck(std::shared_ptr<ConfigSpace> &C, Eigen::Vector2d point);


bool ExplicitPointCheck2D(std::shared_ptr<ConfigSpace> &C,
                          std::shared_ptr<Obstacle> &O,
                          Eigen::VectorXd point,
                          double radius);

bool ExplicitPointCheck(std::shared_ptr<Queue>& Q, Eigen::VectorXd point);

bool ExplicitNodeCheck(std::shared_ptr<Queue>& Q,
                       std::shared_ptr<KDTreeNode> node);

bool LineCheck(std::shared_ptr<ConfigSpace> C,
               std::shared_ptr<KDTree> Tree,
               std::shared_ptr<KDTreeNode> node1,
               std::shared_ptr<KDTreeNode> node2);

#endif // OBSTACLE_H
