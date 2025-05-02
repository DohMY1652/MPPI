#include "mppi_car/Visualizer.h"

Visualizer::Visualizer(ros::NodeHandle& nh, const YAML::Node& visualizing_config, const YAML::Node& control_config)
    : nh(nh), visualizing_config(visualizing_config), control_config(control_config) {

        trajectory_pub = nh.advertise<visualization_msgs::Marker>("trajectory_marker", 1);
        boundary_pub   = nh.advertise<visualization_msgs::Marker>("boundary_marker", 1);

        

        traj_scale = visualizing_config["trajectory_marker"]["scale"].as<double>();
        traj_color.r = visualizing_config["trajectory_marker"]["color"]["r"].as<double>();
        traj_color.g = visualizing_config["trajectory_marker"]["color"]["g"].as<double>();
        traj_color.b = visualizing_config["trajectory_marker"]["color"]["b"].as<double>();
        traj_color.a = visualizing_config["trajectory_marker"]["color"]["a"].as<double>();

        lane_width = control_config["circle_trajectory"]["lane_width"].as<double>();
        boundary_scale = visualizing_config["boundary_marker"]["scale"].as<double>();
        boundary_color.r = visualizing_config["boundary_marker"]["color"]["r"].as<double>();
        boundary_color.g = visualizing_config["boundary_marker"]["color"]["g"].as<double>();
        boundary_color.b = visualizing_config["boundary_marker"]["color"]["b"].as<double>();
        boundary_color.a = visualizing_config["boundary_marker"]["color"]["a"].as<double>();
    }

Visualizer::~Visualizer() {}    

#include "mppi_car/Visualizer.h"

void Visualizer::publish_trajectory(const std::vector<std::vector<double>>& trajectory) {

    visualization_msgs::Marker center_marker;
    center_marker.header.frame_id = "map";
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "trajectory_center";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::Marker::LINE_LIST;
    center_marker.action = visualization_msgs::Marker::ADD;

    center_marker.scale.x = traj_scale;
    center_marker.color = traj_color;
    center_marker.pose.orientation.w = 1.0;

    for (size_t i = 0; i + 2 < trajectory.size(); i += 2) {
        geometry_msgs::Point p1, p2;
        p1.x = trajectory[i][0];
        p1.y = trajectory[i][1];
        p1.z = 0.05;
        p2.x = trajectory[i+1][0];
        p2.y = trajectory[i+1][1];
        p2.z = 0.05;

        center_marker.points.push_back(p1);
        center_marker.points.push_back(p2);
    }

    if (trajectory_pub) trajectory_pub.publish(center_marker);
    else ROS_ERROR("trajectory_pub is not initialized!");

  
    double cx = 0.0, cy = 0.0;
    for (const auto& pt : trajectory) {
        cx += pt[0];
        cy += pt[1];
    }
    cx /= trajectory.size();
    cy /= trajectory.size();

    double r_sum = 0.0;
    for (const auto& pt : trajectory) {
        double dx = pt[0] - cx;
        double dy = pt[1] - cy;
        r_sum += std::sqrt(dx * dx + dy * dy);
    }
    double r = r_sum / trajectory.size();

    for (int offset_sign : {-1, 1}) {
        visualization_msgs::Marker boundary_marker;
        boundary_marker.header.frame_id = "map";
        boundary_marker.header.stamp = ros::Time::now();
        boundary_marker.ns = "trajectory_boundaries";
        boundary_marker.id = offset_sign == -1 ? 1 : 2;
        boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
        boundary_marker.action = visualization_msgs::Marker::ADD;

        boundary_marker.scale.x = boundary_scale;
        boundary_marker.color = boundary_color;
        boundary_marker.pose.orientation.w = 1.0;

        double r_offset = r + offset_sign * lane_width;

        for (const auto& pt : trajectory) {
            double theta = std::atan2(pt[1] - cy, pt[0] - cx);

            geometry_msgs::Point p;
            p.x = cx + r_offset * std::cos(theta);
            p.y = cy + r_offset * std::sin(theta);
            p.z = 0.05;

            boundary_marker.points.push_back(p);
        }

        if (boundary_pub) boundary_pub.publish(boundary_marker);
        else ROS_ERROR("boundary_pub is not initialized!");
    }
}
