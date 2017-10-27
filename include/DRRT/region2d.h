#ifndef REGION2D_H
#define REGION2D_H

#include <DRRT/libraries.h>

class Region2D{
    Eigen::MatrixX2d region_;

public:
    Region2D(Eigen::MatrixX2d r) : region_(r) {}
    Region2D() : region_(Eigen::MatrixX2d()) {}

    Eigen::MatrixX2d GetGlobalPose2D(Eigen::Array2d origin);
    Region2D GetGlobalRegion(Eigen::Array2d origin) { return Region2D(GetGlobalPose2D(origin)); }
    Eigen::MatrixX2d GetRegion() { return region_; }
    void SetRegion(Eigen::MatrixX2d poly) { region_ = poly; }
};

#endif // REGION2D_H
