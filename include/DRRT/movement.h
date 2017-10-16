#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <DRRT/drrt.h>

void Move(CSpace_ptr &cspace, double hyper_ball_rad);
void MovementThread(CSpace_ptr cspace);

#endif // MOVEMENT_H
