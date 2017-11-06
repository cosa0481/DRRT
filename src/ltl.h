#ifndef LTL_H
#define LTL_H

#include <DRRT/drrt.h>
#include <DRRT/visualizer.h>

Robot_ptr Ltl(Problem p);
void ReadStaticObstaclesFromFile(std::string obs_file,
                                 CSpace_ptr cspace);

#endif // LTL_H
