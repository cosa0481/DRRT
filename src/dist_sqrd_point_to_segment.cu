#include "DRRT/dist_sqrd_point_to_segment.cuh"

__global__ void CUDADistanceSquaredPointToSegment(double* query_point_x,
                                                  double* query_point_y,
                                                  double* seg_start_x,
                                                  double* seg_start_y,
                                                  double* seg_end_x,
                                                  double* seg_end_y,
                                                  double* dist)
{
    int idx = threadIdx.x;
    double qpx = *query_point_x;
    double qpy = *query_point_y;
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
        dist[idx] = vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            dist[idx] = (epx-qpx)*(epx-qpx) + (epy-qpy)*(epy-qpy);
        } else {
            dist[idx] = (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}

std::vector<double> CalcDistanceSquaredPointToSegment(
        std::vector<double> query_point,
        std::vector<std::vector<double>> starts,
        std::vector<std::vector<double>> ends)
{
    // Number of threads to use
    int grid_dim = 1;
    int block_dim = starts.size();

    // Host memory pointers
    double* h_qp_x = NULL;
    *h_qp_x = query_point[0];
    double* h_qp_y = NULL;
    *h_qp_y = query_point[1];
    double* h_start_x = NULL;
    double* h_start_y = NULL;
    double* h_end_x = NULL;
    double* h_end_y = NULL;
    for(int i = 0; i < (int)starts.size(); i++) {
        h_start_x[i] = starts.at(i).at(0);
        h_start_y[i] = starts.at(i).at(1);
        h_end_x[i] = ends.at(i).at(0);
        h_end_y[i] = ends.at(i).at(1);
    }
    double* h_dist = NULL;

    // Device memory pointers
    double* d_qp_x;
    double* d_qp_y;
    double* d_start_x;
    double* d_start_y;
    double* d_end_x;
    double* d_end_y;
    double* d_dist;

    // Allocate Device memory
    cudaMalloc((void**) &d_qp_x, sizeof(double));
    cudaMalloc((void**) &d_qp_y, sizeof(double));
    cudaMalloc((void**) &d_start_x, sizeof(double) * starts.size());
    cudaMalloc((void**) &d_start_y, sizeof(double) * starts.size());
    cudaMalloc((void**) &d_end_x, sizeof(double) * ends.size());
    cudaMalloc((void**) &d_end_y, sizeof(double) * ends.size());
    cudaMalloc((void**) &d_dist, sizeof(double) * block_dim);

    // Transfer structures to Device
    cudaMemcpy(d_qp_x, h_qp_x, sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_qp_y, h_qp_y, sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_start_x, h_start_x, sizeof(double) * starts.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_start_y, h_start_y, sizeof(double) * starts.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_end_x, h_end_x, sizeof(double) * ends.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_end_y, h_end_y, sizeof(double) * ends.size(),
                               cudaMemcpyHostToDevice);

    // Launch kernel
    CUDADistanceSquaredPointToSegment<<<grid_dim,block_dim>>>(d_qp_x,
                                                                  d_qp_y,
                                                                  d_start_x,
                                                                  d_start_y,
                                                                  d_end_x,
                                                                  d_end_y,
                                                                  d_dist);

    // Transfer result back to Host
    cudaMemcpy(h_dist, d_dist, sizeof(double) * block_dim,
                               cudaMemcpyDeviceToHost);

    // Put results into vector and return for use
    std::vector<double> dists(block_dim);
    for(int i = 0; i < block_dim; i++) {
        dists.at(i) = ((double)h_dist[i]);
    }
    return dists;
}
