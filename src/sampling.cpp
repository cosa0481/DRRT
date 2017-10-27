#include <DRRT/sampling.h>

double RandomDouble(double min, double max)
{
    uniform_real_distribution<double> unid(min, max);
    mt19937 rng; // Mersenn-Twister random number engine
    rng.seed(random_device{}());
    double random_double = unid(rng);
    return random_double;
}

MatrixX6d TriangulatePolygon(Region region)
{
    MatrixX6d triangles;

    // TODO: Generalize for more than 2-D polygons
    Eigen::MatrixX2d polygon = region.GetRegion();

    double vertices[polygon.rows()][2];
    for(int i = 0; i < polygon.rows(); i++) {
        vertices[i][0] = polygon(i, 0);
        vertices[i][1] = polygon(i, 1);
    }

    // Populate the tris matrix with the index of the vertex used to
    // create the triangle in that row
    int tris[50][3];
    int num_tris = triangulate_polygon(polygon.rows(), vertices, tris);

    triangles.resize(num_tris, Eigen::NoChange_t());
    for(int i = 0; i < num_tris; i ++) {
        triangles(i, 0) = polygon(tris[i][0] - 1, 0); // x1
        triangles(i, 1) = polygon(tris[i][0] - 1, 1); // y1
        triangles(i, 2) = polygon(tris[i][1] - 1, 0); // x2
        triangles(i, 3) = polygon(tris[i][1] - 1, 1); // y2
        triangles(i, 4) = polygon(tris[i][2] - 1, 0); // x3
        triangles(i, 5) = polygon(tris[i][2] - 1, 1); // y3
    }

    return triangles; // triangles.row(i) = (x1, y1, x2, y2, x3, y3)
}

Eigen::VectorXd RandomPoint(CSpace_ptr cspace)
{
    // TODO: Generalize for n dimensions
    double rand_var;
    int num_triangles;
    Eigen::VectorXd point(NUM_DIM);

    Region drivable_region;
    {
        lockguard lock(cspace->mutex_);
        drivable_region = cspace->drivable_region_;
    }

    MatrixX6d triangles;
    {
        lockguard lock(cspace->mutex_);
        triangles = TriangulatePolygon(drivable_region);
    }
    num_triangles = triangles.rows();

    Eigen::Vector2d rand;
    Eigen::MatrixX2d points;
    points.resize(num_triangles, Eigen::NoChange_t());
    for(int i = 0; i < num_triangles; i++) {
        rand(0) = RandomDouble(0, 1);
        rand(1) = RandomDouble(0, 1);

        points(i,0) = (1-sqrt(rand(0))) * triangles(i,0)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,2)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,4);
        points(i,1) = (1-sqrt(rand(0))) * triangles(i,1)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,3)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,5);
    }

    rand_var = RandomDouble(0.0, num_triangles);
    point(0) = points(std::floor(rand_var), 0);
    point(1) = points(std::floor(rand_var), 1);
    point(2) = RandomDouble(0.0, 2*PI);
    return point;
}

Kdnode_ptr RandomNode(CSpace_ptr cspace)
{
    return std::make_shared<Kdnode>(RandomPoint(cspace));
}

Kdnode_ptr RandomNodeOrGoal(CSpace_ptr cspace)
{
    double rand = RandomDouble(0, 1);
    if(rand > cspace->goal_sample_prob_)
        return RandomNode(cspace);
    else
        return cspace->goal_node_;
}

Kdnode_ptr RandomNodeOrFromStack(CSpace_ptr cspace)
{
    if(cspace->sample_stack_->GetLength() > 0) {
        KdnodeListNode_ptr node_listnode = std::make_shared<KdnodeListNode>();
        Kdnode_ptr node = std::make_shared<Kdnode>();
        cspace->sample_stack_->Pop(node_listnode);
        node_listnode->GetData(node);
        return node;
    } else
        return RandomNodeOrGoal(cspace);
}

Kdnode_ptr RandomSampleFromObstacle(CSpace_ptr cspace, Obstacle_ptr obs)
{
    double rand_var;
    int num_triangles;
    Eigen::VectorXd point(NUM_DIM);

    Region sample_region = Region(obs->GetShape().GetGlobalPose2D(obs->GetOrigin()));

    MatrixX6d triangles;
    triangles = TriangulatePolygon(sample_region);
    num_triangles = triangles.rows();

    Eigen::Vector2d rand;
    Eigen::MatrixX2d points;
    points.resize(num_triangles, Eigen::NoChange_t());
    for(int i = 0; i < num_triangles; i++) {
        rand(0) = RandomDouble(0, 1);
        rand(1) = RandomDouble(0, 1);

        points(i,0) = (1-sqrt(rand(0))) * triangles(i,0)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,2)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,4);
        points(i,1) = (1-sqrt(rand(0))) * triangles(i,1)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,3)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,5);
    }

    rand_var = RandomDouble(0, num_triangles);
    point(0) = points(std::floor(rand_var), 0);
    point(1) = points(std::floor(rand_var), 1);
    point(2) = RandomDouble(-2*PI, 2*PI);
    return std::make_shared<Kdnode>(point);
}
