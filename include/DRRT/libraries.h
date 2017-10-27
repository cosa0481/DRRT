#ifndef LIBRARIES_H
#define LIBRARIES_H

#include <Eigen/Eigen>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <cstdlib>
#include <algorithm>
#include <random>
#include <chrono>
#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>

#include <DRRT/distance_functions.h>
inline double DistanceFunction(Eigen::VectorXd a, Eigen::VectorXd b)
{ return DubinsDistance(a,b); }

typedef std::lock_guard<std::mutex> lockguard;

// Triangles container
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;

#define PI 3.1415926536     // Rounded value for pi
#define INF 1e12   // value for infinity
#define NSPS 1e9     // nanoseconds per second
#define MAXPATHNODES 500    // Maximum number of trajectory nodes
#define MAXOBSPOINTS 10     // Maximum number of obstacle vertices
#define NUM_DIM 3           // 0:x 1:y 2:theta

#define DEBUG false          // Debugging print statements
#define DEBUGBULLET false

#endif // LIBRARIES_H
