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
        Visualizer(ros::NodeHandle& nh, const YAML::Node& config);
        ~Visualizer();

        void publish_pose(double x, double y, double theta);
        void publish_velocity_text(double x, double y, double velocity);
        void publish_path(double x, double y, double velocity);
        void publish_goal_markers(const std::vector<double>& goal);
        void publish_lane();

        void update_monitor(std::vector<double> state, std::vector<double> goals);

        void update_sampled_path(std::vector<std::vector<std::vector<double>>> sampled_states);

        int get_sample_number() const;

    private:
        YAML::Node config;

        double lane_start_x_;
        double lane_end_x_;
        double lane_step_;
        double lane_gap_;

        ros::Publisher pose_pub_;
        ros::Publisher velocity_marker_pub_;
        ros::Publisher goal_marker_pub_;
        ros::Publisher path_pub_;
        ros::Publisher center_line_pub_;
        ros::Publisher lane_bounds_pub_;
        ros::Publisher sampled_path_pub_;


        std::vector<geometry_msgs::Point> path_points_;
        std::vector<std_msgs::ColorRGBA> path_colors_;
        int max_path_size_;
        int sample_number_;

        visualization_msgs::Marker center_line_;
        visualization_msgs::Marker lane_bounds_;

        void load_marker_config(const YAML::Node& config);


};

#endif //VISUALIZER_H