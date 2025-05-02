#include "mppi_car/TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig)
: nh(nh), databaseconfig(databaseconfig), previous_index(0) {
    circle_trajectory_parameters = databaseconfig->get_circle_trajectory_parameters();
    global_trajectory.resize((int)circle_trajectory_parameters[3], {0.0, 0.0});
    generator_global_trajectory();
    N = databaseconfig->get_N();
    local_trajectory.resize(N, {0.0, 0.0});
}

TrajectoryPlanner::~TrajectoryPlanner() {}

std::vector<std::vector<double>> TrajectoryPlanner::get_global_trajectory() const {return global_trajectory; }
std::vector<std::vector<double>> TrajectoryPlanner::get_local_trajectory(std::vector<double> now_state) {
    
    double min_dist = std::numeric_limits<double>::max();
    int closest_index = previous_index;

    double now_x = now_state[0];
    double now_y = now_state[1];

    for (int i = previous_index; i < global_trajectory.size(); ++i) {
        double dx = global_trajectory[i][0] - now_x;
        double dy = global_trajectory[i][1] - now_y;
        double dist = dx * dx + dy * dy;  
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    for (int i = 0; i < N; ++i) {
        int idx = closest_index + i;

        if (idx >= global_trajectory.size()) {
            local_trajectory.push_back(global_trajectory.back());
        } else {
            local_trajectory.push_back(global_trajectory[idx]);
        }
    }

    previous_index = closest_index;

    return local_trajectory;
}



//private method

void TrajectoryPlanner::generator_global_trajectory() {
    double x_center = circle_trajectory_parameters[0];
    double y_center = circle_trajectory_parameters[1];
    double radius = circle_trajectory_parameters[2];
    double index = circle_trajectory_parameters[3];
    
    for (int i = 0; i < index; ++i) {
        double theta = 2.0 * M_PI * i / index;
        double x = x_center + radius * std::cos(theta);
        double y = y_center + radius * std::sin(theta);
        global_trajectory[i] = {x, y};
    }
}