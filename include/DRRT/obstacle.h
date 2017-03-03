#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <mutex>
#include <string>

// Infinity value for distance
#define INF 1000000000000
#define MAXPATHNODES 100000
#define DELTA 10 // should be changed if delta is changed in executable

class Obstacle
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
    Obstacle(int kind, Eigen::MatrixX2d polygon)
        : kind_(kind), start_time_(0.0), life_span_(INF),
          obstacle_used_(false), sensible_obstacle_(false),
          obstacle_used_after_sense_(false), polygon_(polygon)
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

    // Functions
    static void ReadObstaclesFromFile(std::string obstacle_file);
};

#endif // OBSTACLE_H
