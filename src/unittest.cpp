/* unittest.cpp
 * Corin Sandford
 * Fall 2016
 * This file is made into an executable: 'test'
 * Used to test basic functionality
 * Should create dense graph and then display optimal path to goal
 * **Should be straight line** !!
 */

#include <DRRT/drrt.h>

using namespace std;

double distanceFunction( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(2) - y.head(2);
    temp = temp*temp;
    return sqrt( temp.sum()
                 + pow( std::min( std::abs(x(3)-y(3)),
                                  std::min(x(3),y(3)) + 2.0*PI
                                  - std::max(x(3),y(3)) ), 2 ) );
}

int main(int argc, char* argv[])
{
    string algorithmName = "RRTx";       // "RRT", "RRT*", "RRT#", or "RRTx"
    string expName = "Debug";            // Name for output files

    double changeThresh = 1.0;           // only for RRTx
    double total_time = 100.0;           // total planning time (move after
                                         // this, and keep planning *50*
    double goal_threshold = 0.5;         // threshold at which the robot has
                                         // reached the goal
    double slice_time = 1.0/100.0;       // for saving data
    double envRad = 50.0;                // environment spans -envRad
                                         // to envRad in each dimension
    double robotRad = 0.5;               // robot radius
    double(*distFunc)(Eigen::VectorXd a, Eigen::VectorXd b)
            = distanceFunction; // distance function for KD-Tree

    Eigen::VectorXd start(4), goal(4);
    start << 0.0,0.0,0.0,PI/4;           // robot goes to *0,-40,0,pi/3*
                                         // (start location of search tree)
    goal << 50.0,50.0,0.0,-3*PI/4 ;      // robot comes from *-40,40,0,-pi/3*
                                         // (goal location of search tree)

    bool MoveRobot = true;

    int d = 4;  // number of dimensions [x y 0.0(time) theta]


    //double timeOut = INF;    // not used?

    Eigen::VectorXd lowerBounds(4), upperBounds(4);
    lowerBounds << -envRad, -envRad, 0.0, 0.0;
    upperBounds << envRad, envRad, 0.0, 2*PI;

    std::shared_ptr<CSpace> C = std::make_shared<CSpace>(d,
                                                         lowerBounds,
                                                         upperBounds,
                                                         start, goal);

    C->robotRadius = robotRad;               // init robot radius
    C->robotVelocity = 10.0;                 // init robot velocity
    C->minTurningRadius = 1.0;               // init robot turning radius
    C->randNode = "randNodeOrFromStack(C)";  // set up sampling function
                                // randNodeOrFromStack( C );
                                // this is hard coded in drrt::RRTX()
    C->pGoal = 0.01;            // probability for not picking random node
    C->spaceHasTime = false;                 // not using time parameter
    C->spaceHasTheta = true;                 // using theta (yaw)

    std::cout << "Parameters defined\nRunning RRTx" << std::endl;

    RRTX(C, total_time, slice_time, 10.0, 100.0, changeThresh,
         algorithmName, MoveRobot, distFunc,
         goal_threshold);

    return 0;
}
