/* test.cpp
 * Corin Sandford
 * Fall 2016
 * This file is made into an executable: 'test'
 * Used to test high level things in the library
 * To define distance function just implement :
 *      'double distanceFunction(Eigen::VectorXd x, Eigen::VectorXd y)'
 * above the main function
 */

#include <DRRT/drrt.h>

using namespace std;

int histPos = 0;
Eigen::MatrixXd rHist(MAXPATHNODES,4);
int kdTreePos = 0;
Eigen::MatrixXd kdTree(MAXPATHNODES,4);
int kdEdgePos = 0;
Eigen::MatrixXd kdEdge(MAXPATHNODES,4);

/////////////////////// main (despite the name) ///////////////////////
void RRTX(std::shared_ptr<CSpace> S, double total_planning_time,
          double slice_time, double delta, double ballConstant,
          double changeThresh, std::string searchType, bool MoveRobotFlag,
          double(*distanceFunction)(Eigen::VectorXd a, Eigen::VectorXd b),
          double goal_threshold)
{
    // Used for "sensing" obstacles (should make input)
    //double robotSensorRange = 20.0;

    // Will hold save time to correct for time spent
    // not in the algorithm itself (time in seconds)
    double saveElapsedTime = 0.0;

    /// INITIALIZATION

    Eigen::VectorXi wraps(1);
    wraps(0) = 3;
    Eigen::VectorXd wrapPoints(1);
    wrapPoints(0) = 2.0*PI;

    /// KDTREE
    // Dubin's car so 4th dimension wraps at 0 = 2pi
    // DEFAULT DISTANCE METRIC (was "EuclideanDist") R3SDist
    std::shared_ptr<KDTree> KD = std::make_shared<KDTree>(S->d, wraps,
                                                          wrapPoints);
    KD->setDistanceFunction(distanceFunction); // drrt_distance_functions.h

    /// QUEUE
    std::shared_ptr<Queue> Q = std::make_shared<Queue>();
    if( searchType == "RRT" ) {
        Q->S = S;
        Q->type = searchType;
    } else if( searchType == "RRT*" ) {
        Q->S = S;
        Q->type = searchType;
    } else if( searchType == "RRT#" ) {
        // false >> use priority queue functions (keyQ)
        // sorted based on cost from goal
        Q->Q = new BinaryHeap(false);
        Q->S = S;
        Q->type = searchType;
    } else if( searchType == "RRTx" ) {
        // false >> use priority queue functions (keyQ)
        // sorted based on cost from goal
        Q->Q = new BinaryHeap(false);
        // Obstacle successor stack
        // true >> uses KDTreeNode's
        Q->OS = std::make_shared<JList>(true);
        Q->S = S;
        Q->changeThresh = changeThresh;
        Q->type = searchType;
    } else {
        std::cout << "Unknown search type: " << searchType << std::endl;
        exit(1);
    }

    // stores a stack of points that we desire
    // to insert in the future (used when an
    // obstacle is removed) true >> uses KDTreeNodes
    S->sampleStack = std::make_shared<JList>(true);

    S->delta = delta;

    double robotRads = S->robotRadius;

    // Define root node in the search tree graph
    std::shared_ptr<KDTreeNode> root
            = std::make_shared<KDTreeNode>(S->start);

    // Explicit check root
    //explicitNodeCheck(S,root);

    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;

    // Insert the root into the KDTree
    KD->kdInsert(root);

    // Define a goal node
    std::shared_ptr<KDTreeNode> goal
            = std::make_shared<KDTreeNode>(S->goal);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
    S->goalNode = goal;
    S->root = root;

    S->moveGoal = goal; // this will store a node at least as far from
                        // the root as the robot. During movement its key
                        // is used to limit propogation beyond the region
                        // we care about

    S->moveGoal->isMoveGoal = true;

    /// ROBOT DATA
    // Parameters that have to do with the robot path following simulation
    std::shared_ptr<RobotData> R
            = std::make_shared<RobotData>(S->goal, goal, MAXPATHNODES);

    double sliceCounter = 0; // helps with saving accurate time data

    if( S->spaceHasTime ) {
        // Add other "times" to root of tree
        addOtherTimesToRoot(S, KD, goal, root, searchType);
    }

    // End initialization
    cout << "Finished Initialization" << endl;

    /// MAIN PROGRAM LOOP
    // While planning time left, plan (will break out when done)
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime
            = std::chrono::high_resolution_clock::now();
    double robot_slice_start
            = std::chrono::duration_cast<std::chrono::nanoseconds>
            (startTime-startTime).count(); // time in nanoseconds
    S->startTimeNs = robot_slice_start; // time in nanoseconds
    S->timeElapsed = 0.0;
    double slice_end_time;

    double oldrrtLMC = INF;
    double now_time = getTimeNs(startTime); // time in nanoseconds
    bool warmUpTimeJustEnded;
    double truncElapsedTime;
    double currentDist;
    Eigen::Vector4d prevPose;

    currentDist = KD->distanceFunction(R->robotPose, root->position);
    prevPose = R->robotPose;

    int i = 0;
    while( true ) {

        double hyperBallRad;
        hyperBallRad = std::min( delta, ballConstant*(
               pow(std::log(1+KD->treeSize)/(KD->treeSize), 1/S->d) ));
        now_time = getTimeNs(startTime); // time in nanoseconds

        // Calculate the end time of the first slice
        slice_end_time = (1+sliceCounter)*slice_time;

        // See if warmup time has ended
        warmUpTimeJustEnded = false;
        if( S->inWarmupTime && S->warmupTime < S->timeElapsed ) {
            warmUpTimeJustEnded = true;
            S->inWarmupTime = false;
        }

        // If this robot has used all of its allotted planning time of this slice
        double now = getTimeNs(startTime); // time in nanoseconds
        S->timeElapsed = (now - S->startTimeNs)/1000000000.0
                       - saveElapsedTime; // time in seconds
        if( S->timeElapsed >= slice_end_time ) {
            cout << "\nIteration: " << i << endl
                 << "------------" << endl;
            i++;
            // Calculate the end time of the next slice
            slice_end_time = (1+sliceCounter)*slice_time;

            robot_slice_start = now_time;
            sliceCounter += 1;
            truncElapsedTime = floor(S->timeElapsed*1000.0)/1000.0;

            /// MOVE ROBOT
            // Move robot if the robot is allowed to move
            /* Changed elapsedTime[checkPtr] to S->timeElapsed
             * since I believe they are the same at this point */
            if( S->timeElapsed > total_planning_time + slice_time ) {
                if( MoveRobotFlag ) {
                    // Assumes Q is rrtxQueue and Q.Q is its BinaryHeap
                    moveRobot(S,Q,KD,slice_time,root,hyperBallRad,R);
                    rHist.row(histPos) = R->robotPose;
                    histPos++;
                } else {
                    cout << "done (robot not moved)" << endl;
                    break;
                }
            }

            // Make graph consistent (RRT# and RRTx)
            if( searchType == "RRTx" || searchType == "RRT#" ) {
                reduceInconsistency(Q, S->moveGoal, robotRads,
                                    root, hyperBallRad);
                if( S->moveGoal->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = S->moveGoal->rrtLMC;
                }
            }

            /// CHECK FOR COMPLETION
            // Check if robot has reached its movement goal
            currentDist = KD->distanceFunction(R->robotPose, root->position);
            cout << "Distance to goal: " << currentDist
                 << " units" << endl;
            if( currentDist < goal_threshold ) {
                cout << "Reached goal" << endl;
                cout << root->position << endl;
                break;
            } else if( KD->distanceFunction(R->robotPose,prevPose) > 10 ) {
                cout << "Impossible move" << endl;
                break;
            }
            prevPose = R->robotPose;

            // START normal graph search stuff
            /// GET RANDOM NODE
            std::shared_ptr<KDTreeNode> newNode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<KDTreeNode> closestNode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> closestDist
                    = std::make_shared<double>(INF);

            newNode = randNodeOrFromStack( S );

            // Happens when we explicitly sample the goal every so often
            if( newNode.get()->kdInTree ) continue;

            // Find closest node already in the graph to the new node
            KD->kdFindNearest(closestNode, closestDist, newNode->position);

            /// SATURATE NODE
            if( *closestDist > delta && newNode != S->goalNode ) {
                double thisDist = KD->distanceFunction(newNode->position,
                                                       closestNode->position);
                std::shared_ptr<Eigen::Vector4d> position
                        = std::make_shared<Eigen::Vector4d>(newNode->position);
                KDTree::saturate(position, closestNode->position,
                                 DELTA, thisDist);
                newNode->position = *position;
            }

            ///*** ASSUMING NO OBSTACLES FOR NOW ***///
            // Check for collisions vs obstacles
            //bool explicitlyUnSafe;
            //explicitNodeCheck( explicitlyUnSafe, S, newNode );
            //if( explicitlyUnSafe ) {
            //    continue;
            //}

            /// EXTEND GRAPH
            extend(S, KD, Q, newNode, closestNode, delta,
                   hyperBallRad, S->moveGoal);
            kdTree.row(kdTreePos) = newNode.get()->position;
            kdTreePos++;

            // Make graph consistent (RRT# and RRTx)
            if( searchType == "RRTx" || searchType == "RRT#" ) {
                reduceInconsistency(Q, S->moveGoal, robotRads,
                                    root, hyperBallRad);
                if( S->moveGoal.get()->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = S->moveGoal.get()->rrtLMC;
                }
            }
        }
    }

    /// SAVE DATA

    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = R->robotMovePath.block( 0,0, R->numRobotMovePoints-1,4 );
    lastpoints = R->robotMovePath.block( 1,0, R->numRobotMovePoints-1,4 );
    diff = firstpoints - lastpoints;
    diff = diff*diff;
    for( int i = 0; i < diff.rows(); i++ ) {
        diff.col(0)(i) = diff.row(i).sum();
    }

    double moveLength = diff.col(0).sqrt().sum();
    std::cout << "Robot traveled: " << moveLength
              << " units" << std::endl;

    double totalTime = getTimeNs(startTime);
    std::cout << "Total time: " << totalTime/1000000000.0
              << " s" << std::endl;

    std::ofstream ofs;
    ofs.open( "robotPath.txt", std::ofstream::out );
    for( int j = 0; j < rHist.rows(); j++ ) {
        ofs << rHist.row(j);
        ofs << "\n";
    }
    ofs.close();

    ofs.open("kdTree.txt", std::ofstream::out );
    for( int k = 0; k < kdTree.rows(); k++ ) {
        ofs << kdTree.row(k);
        ofs << "\n";
    }
    ofs.close();

    std::cout << "Data written to debug/kdTree.txt and debug/robotPath.txt"
              << std::endl;
}

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
    std::cout << "Begin" << std::endl;
    string algorithmName = "RRTx";       // "RRT", "RRT*", "RRT#", or "RRTx"
    string expName = "Debug";            // Name for output files

    double changeThresh = 1.0;           // only for RRTx
    double total_time = 5.0;             // total planning time (move after
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
    start << 0.0,0.0,0.0,-3*PI/4;           // robot goes to *0,-40,0,pi/3*
                                         // (start location of search tree)
    goal << 50.0,50.0,0.0,-3*PI/4 ;      // robot comes from *-40,40,0,-pi/3*
                                         // (goal location of search tree)

    bool MoveRobot = true;

    int d = 4;  // number of dimensions [x y 0.0(time) theta]

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

    RRTX(C,
         total_time,        // total_planning_time
         slice_time,        // slice_time
         10.0,              // delta (max dist between two nodes)
         100.0,             // ballConstant
         changeThresh,      // graph change threshold
         algorithmName,     // algorithmName
         MoveRobot,         // Is the robot moving now
         distFunc,          // distance function to use in the algorithm
         goal_threshold);   // distance to goal node to consider 'done'

    return 0;
}
