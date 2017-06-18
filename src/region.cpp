#include <DRRT/region.h>

Eigen::MatrixX2d Region::GetGlobalPose(Eigen::Array2d origin) {
    Eigen::Array2d segment;
    Eigen::MatrixX2d global_region;
    global_region.resize(this->region_.rows(),Eigen::NoChange_t());
    for(int i = 0; i < this->region_.rows(); i++) {
        segment = this->region_.row(i);
        global_region.row(i) = origin + segment;
    }
    return global_region;
}
