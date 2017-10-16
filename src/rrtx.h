#ifndef RRTX_H
#define RRTX_H

#include <DRRT/drrt.h>

Robot_ptr Rrtx(Problem p);
void ReadObstaclesFromFile(std::string obs_file,
                           CSpace_ptr cspace);

#endif // RRTX_H
