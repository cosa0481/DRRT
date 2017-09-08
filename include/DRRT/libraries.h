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

#define PI 3.1415926536     // Rounded value for pi
#define INF 1000000000000   // 1e12
#define MAXPATHNODES 1000   // Maximum number of RRT nodes
#define MAXOBSPOINTS 10     // Maximum number of obstacle vertices
#define NUM_DIM 3           // 0:x 1:y 2:theta

#define DEBUG true          // Debugging print statements

#endif // LIBRARIES_H
