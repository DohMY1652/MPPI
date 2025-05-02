#include "ros/ros.h"
#include "mppi_car/Dynamics.h"
#include "mppi_car/MPPI.h"
#include "mppi_car/DatabaseConfig.h"
#include "mppi_car/Visualizer.h"
#include "mppi_car/TrajectoryPlanner.h"

#include <vector>
#include <algorithm>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mppi_controller");
    ros::NodeHandle nh;

    std::string control_parameters_yaml_file;
    if (!nh.getParam("control_parameters_yaml_file", control_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'control_parameters_yaml_file'");
        return 1;
    }
    YAML::Node control_config = YAML::LoadFile(control_parameters_yaml_file);

    std::string visualizing_parameters_yaml_file;
    if (!nh.getParam("visualizing_parameters_yaml_file", visualizing_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'visualizing_parameters_yaml_file'");
        return 1;
    }
    YAML::Node visualzing_config = YAML::LoadFile(visualizing_parameters_yaml_file);

    std::shared_ptr<DatabaseConfig> control_databaseconfig = std::make_shared<DatabaseConfig>(control_config);

    std::shared_ptr<TrajectoryPlanner> trajectoryplanner = std::make_shared<TrajectoryPlanner>(nh, control_databaseconfig);
    std::shared_ptr<Visualizer> visualizer = std::make_shared<Visualizer>(nh, visualzing_config, control_config);
    // std::unique_ptr<MPPI> controller = std::make_unique<MPPI>(nh, control_databaseconfig);
    // std::shared_ptr<Dynamics> real_car = std::make_shared<Dynamics>(control_databaseconfig);

    ros::Rate rate((int)control_databaseconfig->get_simulation_frequency());

    
    while (ros::ok()) {
        visualizer->publish_trajectory(trajectoryplanner->get_global_trajectory());

        // if (controller->check_arrival(real_car->get_state())) {
        //     ROS_INFO("Arrival!!");
        //     return 0;
        // } else {
        //     controller->calculate_control(real_car->get_state());
        //     real_car->update_state(controller->get_control());
        // }
        
        // visualizer->update_monitor(real_car->get_state(), control_databaseconfig->get_goals());
        // controller->get_states(100);
        // visualizer->update_sampled_path(controller->get_states(visualizer->get_sample_number()));


        ROS_INFO("running");
        rate.sleep();
    }

    return 0;
}