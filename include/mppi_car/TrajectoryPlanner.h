#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPALNNER_H

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "mppi_car/DatabaseConfig.h"

class TrajectoryPlanner {
    public:
        TrajectoryPlanner(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig);
        ~TrajectoryPlanner();

        std::vector<std::vector<double>> get_global_trajectory() const;

        std::vector<std::vector<double>> get_local_trajectory(std::vector<double> now_state);

    private:
        ros::NodeHandle& nh;
        std::shared_ptr<DatabaseConfig>& databaseconfig;

        std::vector<double> circle_trajectory_parameters;

        std::vector<std::vector<double>> global_trajectory;
        std::vector<std::vector<double>> local_trajectory;

        int N;
        
        int previous_index;


        void generator_global_trajectory();



};

#endif //TRAJECTORY_PLANNER_H