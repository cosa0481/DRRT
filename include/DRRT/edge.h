#ifndef EDGE_H
#define EDGE_H

#include <DRRT/libraries.h>

class Kdnode;
class Obstacle;
class ConfigSpace;

class Edge
{
    std::string type_;

    std::shared_ptr<Kdnode> start_;
    std::shared_ptr<Kdnode> end_;
    double dist_;

    Eigen::MatrixXd trajectory_;

public:
    Edge(std::shared_ptr<Kdnode> start, std::shared_ptr<Kdnode> end)
        : start_(start), end_(end)
    { trajectory_ = Eigen::MatrixXd::Zero(MAXPATHNODES, NUM_DIM); }

    Edge() : dist_(INF) { trajectory_ = Eigen::MatrixXd::Zero(MAXPATHNODES, NUM_DIM); }

    static std::shared_ptr<Edge> NewEdge(std::shared_ptr<Kdnode> start, std::shared_ptr<Kdnode> end);
    static std::shared_ptr<Edge> NewEdge();

    std::string GetType() { return type_; }
    std::shared_ptr<Kdnode> GetStart() { return start_; }
    std::shared_ptr<Kdnode> GetEnd() { return end_; }
    double GetDist() { return dist_; }
    Eigen::MatrixXd GetTrajectory() { return trajectory_; }

    void SetType(std::string type) { type_ = type; }
    void SetStart(std::shared_ptr<Kdnode> start) { start_ = start; }
    void SetEnd(std::shared_ptr<Kdnode> end) { end_ = end; }
    void SetDist(double dist) { dist_ = dist; }
    void SetTrajectory(Eigen::MatrixXd traj) { trajectory_ = traj; }

    // Edge Functions
    virtual bool ValidMove()=0;
    virtual Eigen::VectorXd PoseAtDistAlongEdge(double dist_along_edge)=0;
    virtual void CalculateTrajectory(std::shared_ptr<ConfigSpace> cspace)=0;
    virtual void CalculateHoverTrajectory()=0;
};

typedef std::shared_ptr<Edge> Edge_ptr;

#endif // EDGE_H
