/* test.cpp
 * Corin Sandford
 * Fall 2016
 * This file is made into an executable: 'test'
 * Used to test high level things in the library
 */

#include <DRRT/drrt.h>

using namespace std;

int main( int argc, char* argv[] )
{
    string algorithmName = "RRT";               // "RRT", "RRT*", "RRT#", or "RRTx"
    string expName = "Debug";                   // Name for output files

    double changeThresh = 1.0;                  // only for RRTx

    double total_time = 50.0;                   // total planning time (move after this, and keep planning
    double slice_time = 1.0/10.0;               // for saving data

    double envRad = 50.0;
    double robotRad = 0.5;
    Eigen::VectorXd start, goal;
    start << 0.0, -40.0, 0.0, PI/3;
    goal << -40.0, 40.0, 0.0, -PI/3;

    bool MoveRobot = false;

    int d = 4;
    //double timeOut = INF;

    Eigen::VectorXd lowerBounds, upperBounds;
    lowerBounds << -envRad, -envRad, 0.0, 0.0;
    upperBounds << envRad, envRad, 0.0, 2*PI;

    CSpace* C = new CSpace( d, /*-1.0,*/ lowerBounds, upperBounds, start, goal );

    C->robotRadius = robotRad;
    C->robotVelocity = 2.0;
    C->minTurningRadius = 1.0;

    C->randNode = "randNodeOrFromStack(C)"; //randNodeOrFromStack( C );
    C->pGoal = 0.01;

    C->spaceHasTime = false;
    C->spaceHasTheta = true;

    RRTX( C, total_time, slice_time, 10.0, 100.0, changeThresh, algorithmName, MoveRobot, false, false, "" );

    return 0;
}
