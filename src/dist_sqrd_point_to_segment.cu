#include <stdio.h>

// query_point_x holds all query_point x-values
__global__ double DistanceSqrdPointToSegment(double* query_point_x,
                                             double* query_point_y,
                                             double* seg_start_x,
                                             double* seg_start_y,
                                             double* seg_end_x,
                                             double* seg_end_y)
{
    int idx = 1;//threadIdx.x;
    double qpx = query_point_x[idx];
    double qpy = query_point_y[idx];
    double spx = seg_start_x[idx];
    double spy = seg_start_y[idx];
    double epx = seg_end_x[idx];
    double epy = seg_end_y[idx];
    double vx = qpx - spx;
    double vy = qpy - spy;
    double ux = epx - spx;
    double uy = epy - spy;

    double determinate = vx*ux + vy*uy;

    if( determinate <= 0 ) {
        return vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            return (epx-qpx)*(epx-qpx) + (epy-qpy)*(epy-qpy);
        } else {
            return (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}
