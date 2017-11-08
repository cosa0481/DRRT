#ifndef LTL_H
#define LTL_H

#include <DRRT/drrt.h>
#include <DRRT/theta_star.h>
#include <DRRT/visualizer.h>

Robot_ptr Ltl(Problem p, std::string triangle_file);
MatrixX6d ReadTriangulation(std::string tri_file);
void ReadStaticObstaclesFromFile(std::string obs_file,
                                 CSpace_ptr cspace);

#endif // LTL_H
