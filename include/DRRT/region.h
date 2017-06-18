#ifndef REGION_H
#define REGION_H

#include <DRRT/distancefunctions.h>

class Region{
private:
    Eigen::MatrixX2d region_;

public:
    Region(){
        region_ = Eigen::MatrixX2d();
    }

    Region(Eigen::MatrixX2d r)
        : region_(r) {}

    Eigen::MatrixX2d GetGlobalPose(Eigen::Array2d origin);
    Eigen::MatrixX2d GetPolygon() { return region_; }
    void SetPolygon(Eigen::MatrixX2d polygon) { region_ = polygon; }
};

#endif // REGION_H
