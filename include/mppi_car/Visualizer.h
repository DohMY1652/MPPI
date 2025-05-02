#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2/LinearMath/Quaternion.h"


class Visualizer {
    public:
        Visualizer(ros::NodeHandle& nh, const YAML::Node& visualizing_config, const YAML::Node& control_config);
        ~Visualizer();

        void publish_trajectory(const std::vector<std::vector<double>>& trajectory);

    private:
        ros::NodeHandle& nh;
        YAML::Node visualizing_config;
        YAML::Node control_config;

        ros::Publisher trajectory_pub;
        ros::Publisher boundary_pub;

        double traj_scale;
        std_msgs::ColorRGBA traj_color;
        
        double lane_width;
        double boundary_scale;
        std_msgs::ColorRGBA boundary_color;

};

#endif //VISUALIZER_H