#include <DRRT/region2d.h>

Eigen::MatrixX2d Region2D::GetGlobalPose2D(Eigen::Array2d origin)
{
    Eigen::Array2d segment;
    Eigen::MatrixX2d global_region;
    global_region.resize(region_.rows(), Eigen::NoChange_t());
    for(int i = 0; i < region_.rows(); i++) {
        segment = region_.row(i);
        global_region.row(i) = origin + segment;
    }
    return global_region;
}
