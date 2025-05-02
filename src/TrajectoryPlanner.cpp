#include "mppi_car/TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig)
: nh(nh), databaseconfig(databaseconfig) {
    circle_trajectory_parameters = databaseconfig->get_circle_trajectory_parameters();
    global_trajectory.resize((int)circle_trajectory_parameters[3], {0.0, 0.0});
    generator_global_trajectory();
}

TrajectoryPlanner::~TrajectoryPlanner() {}

std::vector<std::vector<double>> TrajectoryPlanner::get_global_trajectory() const {return global_trajectory; }



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