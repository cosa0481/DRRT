#include <DRRT/obstacle.h>
#include <DRRT/drrt.h>

using namespace std;

void Obstacle::ReadObstaclesFromFile(string obstacle_file,
                                     shared_ptr<ConfigSpace>& C)
{
    // Read a polygon from a file
    ifstream read_stream;
    string line, substring;
    stringstream line_stream;
    int num_polygons, num_points;
    read_stream.open(obstacle_file);
    if(read_stream.is_open()) {
        cout << "Obstacle File: " << obstacle_file << endl;
        // Get number of polygons
        getline(read_stream, line);
        num_polygons = stoi(line);
        line = "";
//        cout << "polygons in file: " << num_polygons << endl;
        for(int i = 0; i < num_polygons; i++) {
            // Get number of points in this polygon
            getline(read_stream, line);
            num_points = stoi(line);
            line = "";
//            cout << "number of vertices: " << num_points << endl;
            Eigen::MatrixX2d polygon;
            if(num_points > 0) polygon.resize(num_points,2);
            for(int i = 0; i < num_points; i++) {
                getline(read_stream,line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                polygon.row(i)(0) = stod(substring);
//                cout << "x: " << polygon.row(i)(0) << endl;
                substring = "";
                getline(line_stream, substring);
                polygon.row(i)(1) = stod(substring);
//                cout << "y: " << polygon.row(i)(1) << endl;
                substring = "";
                line = "";
            }
            getline(read_stream,line);
            line = "";

            shared_ptr<Obstacle> static_polygon
                    = make_shared<Obstacle>(3,polygon,C->space_has_theta_);
            static_polygon->AddObsToConfigSpace(C);
        }
    }
    else { cout << "Error opening obstacle file" << endl; }
    read_stream.close();
    cout << "Read in Obstacles" << endl;
}

void Obstacle::ReadDynamicObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDiscoverableObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDirectionalObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadTimeObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::ReadDynamicTimeObstaclesFromFile(string obstacle_file)
{

}

void Obstacle::UpdateObstacles(shared_ptr<ConfigSpace> &C)
{
    lock_guard<mutex> lock(C->cspace_mutex_);
    shared_ptr<ListNode> obstacle_list_node = C->obstacles_->front_;
    shared_ptr<Obstacle> this_obstacle;
    for(int i = 0; i < C->obstacles_->length_; i++) {
        this_obstacle = obstacle_list_node->obstacle_;
        this_obstacle->DecreaseLife();
        obstacle_list_node = obstacle_list_node->child_;
    }
}

void Obstacle::AddObsToConfigSpace(shared_ptr<ConfigSpace> &C)
{
    lock_guard<mutex> lock(C->cspace_mutex_);
    shared_ptr<Obstacle> this_obstacle = this->GetPointer();
    C->obstacles_->ListPush(this_obstacle);
}

void Obstacle::ChangeObstacleDirection(std::shared_ptr<ConfigSpace> C,
                                       double current_time)
{
    double end_time = C->start_(3);
    // Edges in path ar no longer than this in the time dimension
    double path_time_step = 3.0;

    // Find the path segment of unknown_path that the obstacle is now moving
    // along. Note that future is represented by times closer to S->start_(3)
    // and unknown_path is stored from future (low) to past (high)
    // Nonintuitive I know...
    Eigen::VectorXd high_point, low_point;
    Eigen::ArrayXd temp;
    while(this->next_direction_change_index_ > 0
          && this->unknown_path_.row(this->next_direction_change_index_)(3)
          > current_time) {
        this->next_direction_change_index_ -= 1;
    }

    // Calculate the start and end points of the segment to place in
    // in path and remember next_direction_change_time
    if(this->unknown_path_.row(this->next_direction_change_index_)(3)
            <= current_time
            && this->next_direction_change_index_
            == this->unknown_path_.rows()) {
        // Obstacle has not started moving yet, so assume that
        // we know the first movement segment of the robot
        int index = this->unknown_path_.rows()-1;

        high_point = this->unknown_path_.row(index);
        low_point = this->unknown_path_.row(index-1);
        this->next_direction_change_time_
                = this->unknown_path_.row(index-1)(3);
    } else if(this->unknown_path_.row(this->next_direction_change_index_)(3)
              > current_time
              && this->next_direction_change_index_ <= 1) {
        // Time has progressed further than this obstacle has a path
        // So assume it remains at the end of its path until lifetime expires
        temp << this->unknown_path_.row(0).head(3), current_time;
        high_point = temp;
        temp(3) = current_time-path_time_step;
        low_point = temp;
        this->next_direction_change_time_ = -INF;
    } else {
        high_point = this->unknown_path_.row(
                    this->next_direction_change_index_+1);
        low_point = this->unknown_path_.row(
                    this->next_direction_change_index_);
        this->next_direction_change_time_
                = this->unknown_path_.row(
                    this->next_direction_change_index_)(3);
    }

    // Calculate path, which is a line parallell to the edge
    // [high_point,low_point] and that goes from current_time to end_time in
    // the time dimension
    // The required time must be calculated based on the obstacle's speed

    double mx = (high_point(0) - low_point(0))/(high_point(3) - low_point(3));
    double my = (high_point(1) - low_point(1))/(high_point(3) - low_point(3));

    Eigen::VectorXd ts = Eigen::VectorXd::Zero(ceil(abs(current_time-end_time)
                                                    / path_time_step)+1);

    if(current_time == end_time) {
        ts(0) = current_time;
    } else {
        double val = current_time;
        int count = 0;
        while( count >= end_time ) {
            ts(count) = val;
            val -= path_time_step;
            count++;
        }
        ts(count) = end_time;
    }

    int length = ts.size();
    for(int i = 0; i < length; i++) {
        this->path_.row(i)(0) = low_point(0) + mx*(ts(length-i+1) - low_point(3));
        this->path_.row(i)(1) = low_point(1) + my*(ts(length-i+1) - low_point(3));
        this->path_.row(i)(3) = ts(length-i+1);
    }
}
