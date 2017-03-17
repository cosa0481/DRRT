#ifndef DIST_SQRD_POINT_TO_SEGMENT_CUH
#define DIST_SQRD_POINT_TO_SEGMENT_CUH

#include <stdio.h>
#include <iostream>
#include <vector>

std::vector<double> CalcDistanceSquaredPointToSegment(
        std::vector<double> query_point,
        std::vector<std::vector<double>> starts,
        std::vector<std::vector<double>> ends);

#endif // DIST_SQRD_POINT_TO_SEGMENT_CUH
