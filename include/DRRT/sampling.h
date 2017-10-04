#ifndef SAMPLING_H
#define SAMPLING_H

#include <DRRT/kdtree.h>
#include <DRRT/tripolyinterface.h>
#include <DRRT/data_structures.h>

double RandomDouble(double min, double max);

// Decomposes a polygon into triangles
// Atul Narkhede and Dinesh Manocha : UNC Chapel Hill : 1995
// Returns a matrix with each row containing the x,y coordinates for
// a triangle. The format for a row is
// x1 y1 x2 y2 x3 y3
MatrixX6d TriangulatePolygon(Region region);

Eigen::VectorXd RandomPoint(CSpace_ptr cspace);

Kdnode_ptr RandomNode(CSpace_ptr cspace);

Kdnode_ptr RandomNodeOrGoal(CSpace_ptr cspace);

Kdnode_ptr RandomNodeOrFromStack(CSpace_ptr cspace);

Kdnode_ptr RandomSampleFromObstacle(CSpace_ptr cspace, Obstacle_ptr obs);

#endif // SAMPLING_H
