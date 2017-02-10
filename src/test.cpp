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

std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

/// MAIN CONTROL FUNCTION
// This function runs RRTx with the parameters defined in main()
std::shared_ptr<RobotData> RRTX(Problem p)
{
    if(p.search_type != "RRTx") {
        std::cout << "y u no want RRTx?" << std::endl;
        exit(1);
    }

    // Used for "sensing" obstacles (should make input)
    //double robotSensorRange = 20.0;

    // Will hold save time to correct for time spent
    // not in the algorithm itself (time in seconds)
    double saveElapsedTime = 0.0;

    /// INITIALIZATION

    /// KDTREE
    std::shared_ptr<KDTree> kdtree
            = std::make_shared<KDTree>(p.c_space->d, p.wraps,
                                       p.wrap_points);
    kdtree->setDistanceFunction(p.distance_function);

    /// QUEUE
    std::shared_ptr<Queue> Q = std::make_shared<Queue>();
    if( p.search_type == "RRT" ) {
        Q->S = p.c_space;
        Q->type = p.search_type;
    } else if( p.search_type == "RRT*" ) {
        Q->S = p.c_space;
        Q->type = p.search_type;
    } else if( p.search_type == "RRT#" ) {
        // false >> use priority queue functions (keyQ)
        // sorted based on cost from goal
        Q->Q = new BinaryHeap(false);
        Q->S = p.c_space;
        Q->type = p.search_type;
    } else if( p.search_type == "RRTx" ) {
        // false >> use priority queue functions (keyQ)
        // sorted based on cost from goal
        Q->Q = new BinaryHeap(false);
        // Obstacle successor stack
        // true >> uses KDTreeNode's
        Q->OS = std::make_shared<JList>(true);
        Q->S = p.c_space;
        Q->changeThresh = p.change_threshold;
        Q->type = p.search_type;
    } else {
        std::cout << "Unknown search type: " << p.search_type << std::endl;
        exit(1);
    }

    // stores a stack of points that we desire
    // to insert in the future (used when an
    // obstacle is removed) true >> uses KDTreeNodes
    p.c_space->sampleStack = std::make_shared<JList>(true);

    p.c_space->delta = p.delta;

    double robotRads = p.c_space->robotRadius;

    // Define root node in the search tree graph
    std::shared_ptr<KDTreeNode> root
            = std::make_shared<KDTreeNode>(p.c_space->start);

    // Explicit check root
    //explicitNodeCheck(S,root);

    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;

    // Insert the root into the KDTree
    kdtree->kdInsert(root);

    // Define a goal node
    std::shared_ptr<KDTreeNode> goal
            = std::make_shared<KDTreeNode>(p.c_space->goal);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
    p.c_space->goalNode = goal;
    p.c_space->root = root;

    p.c_space->moveGoal = goal; // this will store a node at least as far from
                                // the root as the robot. During movement
                                // its key is used to limit propogation
                                // beyond the region we care about

    p.c_space->moveGoal->isMoveGoal = true;

    /// ROBOT DATA
    // Parameters that have to do with the robot path following simulation
    std::shared_ptr<RobotData> robot
            = std::make_shared<RobotData>(p.c_space->goal, goal, MAXPATHNODES);

    double sliceCounter = 0; // helps with saving accurate time data

    if( p.c_space->spaceHasTime ) {
        // Add other "times" to root of tree
        addOtherTimesToRoot(p.c_space, kdtree, goal, root, p.search_type);
    }

    // End initialization
    cout << "Finished Initialization" << endl;

    /// MAIN PROGRAM LOOP
    // While planning time left, plan (will break out when done)
    startTime = std::chrono::high_resolution_clock::now();
    double robot_slice_start
            = std::chrono::duration_cast<std::chrono::nanoseconds>
            (startTime-startTime).count(); // time in nanoseconds
    p.c_space->startTimeNs = robot_slice_start; // time in nanoseconds
    p.c_space->timeElapsed = 0.0;
    double slice_end_time;

    double oldrrtLMC = INF;
    double now_time = getTimeNs(startTime); // time in nanoseconds
    bool warmUpTimeJustEnded;
    double truncElapsedTime;
    double currentDist;
    Eigen::Vector4d prevPose;

    currentDist = kdtree->distanceFunction(robot->robotPose, root->position);
    prevPose = robot->robotPose;

    int i = 0;
    while( true ) {

        double hyperBallRad;
        hyperBallRad = std::min( p.c_space->delta, p.ball_constant*(
               pow(std::log(1+kdtree->treeSize)/(kdtree->treeSize),
                   1/p.c_space->d) ));
        now_time = getTimeNs(startTime); // time in nanoseconds

        // Calculate the end time of the first slice
        slice_end_time = (1+sliceCounter)*p.slice_time;

        // See if warmup time has ended
        warmUpTimeJustEnded = false;
        if( p.c_space->inWarmupTime
                && p.c_space->warmupTime < p.c_space->timeElapsed ) {
            warmUpTimeJustEnded = true;
            p.c_space->inWarmupTime = false;
        }


        double now = getTimeNs(startTime); // time in nanoseconds
        p.c_space->timeElapsed = (now - p.c_space->startTimeNs)/1000000000.0
                               - saveElapsedTime; // time in seconds
        // If robot has used all of its allotted planning time of this slice
        if( p.c_space->timeElapsed >= slice_end_time ) {
            cout << "\nIteration: " << i << endl
                 << "------------" << endl;
            i++;
            // Calculate the end time of the next slice
            slice_end_time = (1+sliceCounter)*p.slice_time;

            robot_slice_start = now_time;
            sliceCounter += 1;
            truncElapsedTime = floor(p.c_space->timeElapsed*1000.0)/1000.0;

            /// MOVE ROBOT
            // Move robot if the robot is allowed to move
            /* Changed elapsedTime[checkPtr] to S->timeElapsed
             * since I believe they are the same at this point */
            if(p.c_space->timeElapsed > p.planning_only_time + p.slice_time) {
                if( p.move_robot_flag ) {
                    // Assumes Q is rrtxQueue and Q.Q is its BinaryHeap
                    moveRobot(p.c_space,Q,kdtree,root,p.slice_time,
                              hyperBallRad,robot);
                    rHist.row(histPos) = robot->robotPose;
                    histPos++;
                } else {
                    cout << "done (robot not moved)" << endl;
                    break;
                }
            }

            // Make graph consistent (RRT# and RRTx)
            if( p.search_type == "RRTx" || p.search_type == "RRT#" ) {
                reduceInconsistency(Q, p.c_space->moveGoal, robotRads,
                                    root, hyperBallRad);
                if( p.c_space->moveGoal->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = p.c_space->moveGoal->rrtLMC;
                }
            }

            /// CHECK FOR COMPLETION
            // Check if robot has reached its movement goal
            currentDist = kdtree->distanceFunction(robot->robotPose,
                                                   root->position);
            cout << "Distance to goal: " << currentDist
                 << " units" << endl;
            if( currentDist < p.goal_threshold ) {
                cout << "Reached goal" << endl;
                cout << root->position << endl;
                break;
            }
            else if(kdtree->distanceFunction(robot->robotPose,prevPose) > 10) {
                cout << "Impossible move" << endl;
                break;
            }
            prevPose = robot->robotPose;

            // START normal graph search stuff
            /// GET RANDOM NODE
            std::shared_ptr<KDTreeNode> newNode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<KDTreeNode> closestNode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> closestDist
                    = std::make_shared<double>(INF);

            newNode = randNodeOrFromStack( p.c_space );

            // Happens when we explicitly sample the goal every so often
            if( newNode->kdInTree ) continue;

            // Find closest node already in the graph to the new node
            kdtree->kdFindNearest(closestNode, closestDist, newNode->position);

            /// SATURATE NODE
            if( *closestDist > p.c_space->delta
                    && newNode != p.c_space->goalNode ) {
                double thisDist = kdtree->distanceFunction(newNode->position,
                                                       closestNode->position);
                std::shared_ptr<Eigen::Vector4d> position
                        = std::make_shared<Eigen::Vector4d>(newNode->position);
                Edge::saturate(position, closestNode->position,
                                 p.c_space->delta, thisDist);
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
            extend(p.c_space, kdtree, Q, newNode, closestNode,
                   p.c_space->delta, hyperBallRad, p.c_space->moveGoal);
            kdTree.row(kdTreePos) = newNode.get()->position;
            kdTreePos++;

            // Make graph consistent (RRT# and RRTx)
            if( p.search_type == "RRTx" || p.search_type == "RRT#" ) {
                reduceInconsistency(Q, p.c_space->moveGoal, robotRads,
                                    root, hyperBallRad);
                if( p.c_space->moveGoal->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = p.c_space->moveGoal->rrtLMC;
                }
            }
        }
    }
    return robot;
}

/// Distance Function for use in the RRTx algorithm
// This was created by Michael Otte (R3SDist)
double distance_function( Eigen::VectorXd a, Eigen::VectorXd b )
{
    Eigen::ArrayXd temp = a.head(2) - b.head(2);
    temp = temp*temp;
    return sqrt( temp.sum()
                 + pow( std::min( std::abs(a(3)-b(3)),
                                  std::min(a(3),b(3)) + 2.0*PI
                                  - std::max(a(3),b(3)) ), 2 ) );
}

int main(int argc, char* argv[])
{
    /// START, GOAL
    Eigen::VectorXd start(4), goal(4);
    start << 0.0,0.0,0.0,-3*PI/4;        // robot goes here
                                         // (goal location of search tree)
    goal << 50.0,50.0,0.0,-3*PI/4 ;      // robot comes from here
                                         // (start location of search tree)

    /// C-SPACE
    int d = 4; // number of dimensions [x y 0.0(time) theta] (Dubins)

    double envRad = 50.0;
    // environment spans -envRad to envRad in each dimension
    Eigen::Vector4d lowerBounds, upperBounds;
    lowerBounds << -envRad, -envRad, 0.0, 0.0;
    upperBounds << envRad, envRad, 0.0, 2*PI;

    std::shared_ptr<CSpace> config_space
        = std::make_shared<CSpace>(d, lowerBounds, upperBounds, start, goal);

    config_space->robotRadius = 0.5;                   // robot radius
    config_space->robotVelocity = 10.0;                // robot velocity
    config_space->minTurningRadius = 1.0;              // robot turn radius
    ///*** this is hard coded in RRTX() ***///
    config_space->randNode = "randNodeOrFromStack";    // sampling function
    config_space->pGoal = 0.01; // probability of picking goal when sampling

    config_space->spaceHasTime = false;                // not using time
    config_space->spaceHasTheta = true;                // using theta (yaw)

    /// KDTREE VARIABLES
    // Dubin's model wraps theta (4th entry) at 2pi
    Eigen::VectorXi wrap_vec(1); // 1 wrapping dimension
    wrap_vec(0) = 3;
    Eigen::VectorXd wrap_points_vec(1);
    wrap_points_vec(0) = 2.0*PI;

    /// PARAMETERS
    string algorithm_name = "RRTx"; // Running RRTx
    double plan_time = 5.0;         // plan only for this long
    double slice_time = 1.0/100;    // iteration plan time limit
    double delta = 10.0;            // distance between graph nodes
    double ball_const = 100.0;      // search n-ball radius
    double change_thresh = 1.0;     // obstacle change detection
    double goal_thresh = 0.5;       // goal detection
    bool move_robot = true;         // move robot after plan_time/slice_time

    // Create a new problem for RRTx
    Problem problem = Problem(algorithm_name,
                              config_space,
                              plan_time,
                              slice_time,
                              delta,
                              ball_const,
                              change_thresh,
                              goal_thresh,
                              move_robot,
                              wrap_vec,
                              wrap_points_vec,
                              distance_function);

    /// RUN RRTX
    std::shared_ptr<RobotData> robot_data = RRTX(problem);

    /// SAVE DATA
    // Calculate and display distance traveled
    Eigen::ArrayXXd firstpoints, lastpoints, diff;
    firstpoints = robot_data->robotMovePath.block(0,0,
                                      robot_data->numRobotMovePoints-1,4);
    lastpoints = robot_data->robotMovePath.block(1,0,
                                     robot_data->numRobotMovePoints-1,4);
    diff = firstpoints - lastpoints;
    diff = diff*diff;
    for( int i = 0; i < diff.rows(); i++ ) {
        diff.col(0)(i) = diff.row(i).sum();
    }

    double moveLength = diff.col(0).sqrt().sum();
    std::cout << "Robot traveled: " << moveLength
              << " units" << std::endl;

    // Calculate and display time elapsed
    double totalTime = getTimeNs(startTime);
    std::cout << "Total time: " << totalTime/1000000000.0
              << " s" << std::endl;

    std::ofstream ofs;
    // Export robot path to file
    ofs.open( "robotPath.txt", std::ofstream::out );
    for( int j = 0; j < rHist.rows(); j++ ) {
        ofs << rHist.row(j);
        ofs << "\n";
    }
    ofs.close();

    // Export kd-tree to file
    ofs.open("kdTree.txt", std::ofstream::out );
    for( int k = 0; k < kdTree.rows(); k++ ) {
        ofs << kdTree.row(k);
        ofs << "\n";
    }
    ofs.close();

    std::cout << "Data written to debug/kdTree.txt and debug/robotPath.txt"
              << std::endl;

    return 0;
}
