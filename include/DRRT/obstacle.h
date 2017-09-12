#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <DRRT/libraries.h>
#include <DRRT/region.h>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexHullShape.h>

class ConfigSpace;
class KdTree;

class Obstacle : public std::enable_shared_from_this<Obstacle>
{
    int kind_;

    double start_time_;  // Time (secs) after cspace->start_time_ when obstacle exists
    double lifetime_;  // Time (secs) for which obstacle exists

    bool is_used_;
    bool is_sensible_;
    bool is_used_after_sense_;

    Region original_shape_global_;  // Original global coordinates of object shape
    Eigen::VectorXd origin_;  // Global coordinates of origin of obstacle
    Region shape_;  // Shape of the obstacle with respect to the origin
    double radius_;  // Distance to furthest vertex from origin

    std::shared_ptr<btCollisionObject> collision_object_;
    std::shared_ptr<btConvexHullShape> collision_shape_;

    double velocity_;
    Eigen::MatrixXd path_;  // Locations of the origin of obstacle in path
    Eigen::VectorXd path_times_;  // Times when obstacle moves along path in seconds
    int current_path_idx_;  // Current index of path and path times

public:
    std::shared_ptr<ConfigSpace> cspace;  // pointer to cspace where obstacle lives

    Obstacle(int kind) : kind_(kind) {}
    Obstacle(int kind, Eigen::VectorXd origin, Eigen::MatrixX2d shape,
             Eigen::MatrixXd path, Eigen::VectorXd path_times,
             std::shared_ptr<ConfigSpace> configspace)
        : kind_(kind), is_used_(false), is_sensible_(false),
          is_used_after_sense_(false),
          origin_(origin), shape_(Region(shape)),
          collision_object_(std::make_shared<btCollisionObject>()),
          path_(path), path_times_(path_times), current_path_idx_(0),
          cspace(configspace)
    {
        original_shape_global_ = shape_.GetGlobalPose2D(origin);
        double dist, max = 0;
        for(int i = 0; i < shape.rows(); i++) {
            dist = sqrt(pow(shape.row(i)(0) - origin_(0), 2)
                        + pow(shape.row(i)(1) - origin_(1), 2));
            if(dist > max) max = dist;
        }
        radius_ = max;
    }
    Obstacle() : kind_(-1) {}

    std::shared_ptr<Obstacle> GetSharedPointer() { return shared_from_this(); }

    // Getters & Setters
    int GetKind() { return kind_; }
    double GetStartTime() { return start_time_; }
    double GetLifeTime() { return lifetime_; }
    bool IsUsed() { return is_used_; }
    bool IsSensible() { return is_sensible_; }
    bool IsUsedAfterSense() { return is_used_after_sense_; }
    Region GetOriginalShapeGlobal() { return original_shape_global_; }
    Eigen::VectorXd GetOrigin() { return origin_; }
    Region GetShape() { return shape_; }
    double GetRadius() { return radius_; }
    std::shared_ptr<btCollisionObject> GetCollisionObject()
    { return collision_object_; }
    std::shared_ptr<btConvexHullShape> GetCollisionShape()
    { return collision_shape_; }
    double GetVelocity() { return velocity_; }
    Eigen::MatrixXd GetPath() { return path_; }
    Eigen::VectorXd GetPathTimes() { return path_times_; }
    int GetCurrentPathIdx() { return current_path_idx_; }

    void SetKind(int k) { kind_ = k; }
    void SetStartTime(double time) { start_time_ = time; }
    void SetLifeTime(double time) { lifetime_ = time; }
    void SetIsUsed(bool used) { is_used_ = used; }
    void SetIsSensible(bool sensible) { is_sensible_ = sensible; }
    void SetIsUsedAfterSense(bool used) { is_used_after_sense_ = used; }
    void SetOrigin(Eigen::VectorXd origin) { origin_ = origin; }
    void SetShape(Region shape) { shape_ = shape; }
    void SetCollisionObject(std::shared_ptr<btCollisionObject> object)
    { collision_object_ = object; }
    void SetCollisionShape(std::shared_ptr<btConvexHullShape> shape)
    { collision_shape_ = shape; }
    void SetVelocity(double vel) { velocity_ = vel; }
    void SetPath(Eigen::MatrixXd path) { path_ = path; }
    void SetPathTimes(Eigen::VectorXd times) { path_times_ = times; }
    void SetCurrentPathIdx(int idx) { current_path_idx_ = idx; }

    // Obstacle functions
    static void UpdateObstacles();
    static bool MoveObstacles(std::shared_ptr<ConfigSpace> &cspace);

    void UpdatePosition(Eigen::VectorXd new_pos)
    { origin_ = new_pos; }

    bool MoveObstacle();
    void AddObstacle(std::shared_ptr<KdTree> tree);
    void RemoveObstacle(std::shared_ptr<KdTree> tree);

    void AddToCSpace();
    static void ReadObstaclesFromFile(std::string obs_file,
                                      std::shared_ptr<ConfigSpace> cspace);
};

typedef std::shared_ptr<Obstacle> Obstacle_ptr;

#endif // OBSTACLE_H
