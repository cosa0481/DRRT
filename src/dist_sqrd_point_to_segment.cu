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
    int block_dim = starts[0].size();

    // Host memory pointers
//    std::cout << "host pointers" << std::endl;
    double* h_qp_x = nullptr;
    h_qp_x = &(query_point[0]);
    double* h_qp_y = nullptr;
    h_qp_y = &(query_point[1]);
    std::vector<double> h_start_x;
    std::vector<double> h_start_y;
    std::vector<double> h_end_x;
    std::vector<double> h_end_y;
    for(int i = 0; i < block_dim; i++) {
        h_start_x.push_back(starts[i][0]);
        h_start_y.push_back(starts[i][1]);
        h_end_x.push_back(ends[i][0]);
        h_end_y.push_back(ends[i][1]);
    }
    double h_dist[block_dim];

    // Device memory pointers
//    std::cout << "device pointers" << std::endl;
    double* d_qp_x = nullptr;
    double* d_qp_y = nullptr;
    double* d_start_x = nullptr;
    double* d_start_y = nullptr;
    double* d_end_x = nullptr;
    double* d_end_y = nullptr;
    double* d_dist = nullptr;

    // Allocate Device memory
//    std::cout << "cudaMalloc" << std::endl;
    cudaMalloc((void**) &d_qp_x, sizeof(double));
    cudaMalloc((void**) &d_qp_y, sizeof(double));
    cudaMalloc((void**) &d_start_x, sizeof(double) * h_start_x.size());
    cudaMalloc((void**) &d_start_y, sizeof(double) * h_start_y.size());
    cudaMalloc((void**) &d_end_x, sizeof(double) * h_end_x.size());
    cudaMalloc((void**) &d_end_y, sizeof(double) * h_end_y.size());
    cudaMalloc((void**) &d_dist, sizeof(double) * block_dim);

    // Transfer structures to Device
//    std::cout << "cudaMemcpyHostToDevice" << std::endl;
    cudaMemcpy(d_qp_x, h_qp_x, sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_qp_y, h_qp_y, sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_start_x, &h_start_x[0], sizeof(double) * h_start_x.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_start_y, &h_start_y[0], sizeof(double) * h_start_y.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_end_x, &h_end_x[0], sizeof(double) * h_end_x.size(),
                               cudaMemcpyHostToDevice);
    cudaMemcpy(d_end_y, &h_end_y[0], sizeof(double) * h_end_y.size(),
                               cudaMemcpyHostToDevice);

    // Launch kernel
//    std::cout << "kernel" << std::endl;
    CUDADistanceSquaredPointToSegment<<<grid_dim,block_dim>>>(d_qp_x,
                                                              d_qp_y,
                                                              d_start_x,
                                                              d_start_y,
                                                              d_end_x,
                                                              d_end_y,
                                                              d_dist);

    // Transfer result back to Host
//    std::cout << "cudaMemcpyDeviceToHost" << std::endl;
    cudaMemcpy(&h_dist[0], d_dist, sizeof(double) * block_dim,
                               cudaMemcpyDeviceToHost);

    // Put results into vector and return for use
    std::vector<double> dists(block_dim);
    for(int i = 0; i < block_dim; i++) {
        dists.at(i) = ((double)h_dist[i]);
    }
    return dists;
}
