#include "mppi_car/Visualizer.h"


Visualizer::Visualizer(ros::NodeHandle& nh, const YAML::Node& config)
    : config(config) {
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
        velocity_marker_pub_ = nh.advertise<visualization_msgs::Marker>("velocity_marker", 10);
        goal_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("goal_markers", 10);
        path_pub_ = nh.advertise<visualization_msgs::Marker>("path_marker", 10);
        center_line_pub_ = nh.advertise<visualization_msgs::Marker>("center_line_marker", 1);
        lane_bounds_pub_ = nh.advertise<visualization_msgs::Marker>("lane_bounds_marker", 1);
        sampled_path_pub_ = nh.advertise<visualization_msgs::Marker>("sampled_paths", 1);


        max_path_size_ = config["visualization"]["path"]["max_size"].as<int>();
        sample_number_ = config["visualization"]["sample"]["visualizing_number"].as<int>();

        center_line_.header.frame_id = "map";
        center_line_.ns = "center_line";
        center_line_.id = 100;
        center_line_.type = visualization_msgs::Marker::LINE_LIST;
        center_line_.action = visualization_msgs::Marker::ADD;

        lane_bounds_.header.frame_id = "map";
        lane_bounds_.ns = "lane_bounds";
        lane_bounds_.id = 101;
        lane_bounds_.type = visualization_msgs::Marker::LINE_LIST;
        lane_bounds_.action = visualization_msgs::Marker::ADD;

        load_marker_config(config);

        for (double x = lane_start_x_; x < lane_end_x_; x += lane_step_) {
            geometry_msgs::Point p1, p2;
            p1.x = x; p1.y = x; p1.z = 0.05;
            p2.x = x + lane_step_ / 2; p2.y = x + lane_step_ / 2; p2.z = 0.05;
            center_line_.points.push_back(p1);
            center_line_.points.push_back(p2);
    
            // 위 경계
            p1.y = x + lane_gap_; p2.y = x + lane_step_ / 2 + lane_gap_;
            lane_bounds_.points.push_back(p1);
            lane_bounds_.points.push_back(p2);
    
            // 아래 경계
            p1.y = x - lane_gap_; p2.y = x + lane_step_ / 2 - lane_gap_;
            lane_bounds_.points.push_back(p1);
            lane_bounds_.points.push_back(p2);
        }

    }





Visualizer::~Visualizer() {}

void Visualizer::publish_pose(double x, double y, double theta) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);
}

void Visualizer::publish_velocity_text(double x, double y, double velocity) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "velocity_text";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 1.0;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.text = "v = " + std::to_string(velocity);

    velocity_marker_pub_.publish(marker);
}

void Visualizer::publish_path(double x, double y, double velocity) {
    geometry_msgs::Point point;
    point.x = x; point.y = y; point.z = 0.0;
    path_points_.push_back(point);

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    double v_clamped = std::min(std::max(velocity, 0.0), 5.0);
    color.r = v_clamped / 5.0;
    color.b = 1.0 - v_clamped / 5.0;
    path_colors_.push_back(color);

    if (path_points_.size() > max_path_size_) {
        path_points_.erase(path_points_.begin());
        path_colors_.erase(path_colors_.begin());
    }

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.05;
    path_marker.points = path_points_;
    path_marker.colors = path_colors_;
    path_marker.color.a = 1.0;
    path_marker.pose.orientation.w = 1.0;

    path_pub_.publish(path_marker);
}

void Visualizer::publish_goal_markers(const std::vector<double>& goal) {
    if (goal.size() < 4) return;

    double goal_x = goal[0];
    double goal_y = goal[1];
    double goal_theta = goal[2];
    double goal_range = goal[3];

    visualization_msgs::MarkerArray marker_array;

    // 구
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "map";
    point_marker.ns = "goal_points";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = goal_x;
    point_marker.pose.position.y = goal_y;
    point_marker.pose.position.z = 0.2;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.scale.z = 0.2;
    point_marker.color.r = 1.0;
    point_marker.color.a = 1.0;

    marker_array.markers.push_back(point_marker);

    // 범위 원
    visualization_msgs::Marker circle_marker;
    circle_marker.header.frame_id = "map";
    circle_marker.ns = "goal_ranges";
    circle_marker.id = 1;
    circle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    circle_marker.action = visualization_msgs::Marker::ADD;
    circle_marker.pose.orientation.w = 1.0;

    int num_points = 50;
    for (int j = 0; j <= num_points; ++j) {
        double angle = 2.0 * M_PI * j / num_points;
        geometry_msgs::Point p;
        p.x = goal_x + goal_range * cos(angle);
        p.y = goal_y + goal_range * sin(angle);
        p.z = 0.1;
        circle_marker.points.push_back(p);
    }

    circle_marker.scale.x = 0.05;
    circle_marker.color.r = 1.0;
    circle_marker.color.g = 1.0;
    circle_marker.color.b = 0.0;
    circle_marker.color.a = 1.0;
    marker_array.markers.push_back(circle_marker);

    goal_marker_pub_.publish(marker_array);
}

void Visualizer::publish_lane() {
    ros::Time stamp = ros::Time::now();
    center_line_.header.stamp = stamp;
    lane_bounds_.header.stamp = stamp;
    center_line_pub_.publish(center_line_);
    lane_bounds_pub_.publish(lane_bounds_);
}

void Visualizer::update_monitor(std::vector<double> state, std::vector<double> goals) {
        double x = state[0], y = state[1], theta = state[2], velocity = state[3];
        publish_goal_markers(goals);
        publish_pose(state[0], state[1], state[2]);
        publish_velocity_text(state[0], state[1], state[3]);
        publish_path(state[0], state[1], state[3]);
        publish_lane();
}

void Visualizer::load_marker_config(const YAML::Node& config) {
    auto vis = config["visualization"];
    
    // Center Line
    auto c_color = vis["center_line"]["color"];
    center_line_.color.r = c_color[0].as<double>();
    center_line_.color.g = c_color[1].as<double>();
    center_line_.color.b = c_color[2].as<double>();
    center_line_.color.a = c_color[3].as<double>();
    center_line_.scale.x = vis["center_line"]["scale"].as<double>();

    // Lane Bounds
    auto l_color = vis["lane_bounds"]["color"];
    lane_bounds_.color.r = l_color[0].as<double>();
    lane_bounds_.color.g = l_color[1].as<double>();
    lane_bounds_.color.b = l_color[2].as<double>();
    lane_bounds_.color.a = l_color[3].as<double>();
    lane_bounds_.scale.x = vis["lane_bounds"]["scale"].as<double>();

    // Geometry (range)
    lane_start_x_ = vis["lane_range"]["start_x"].as<double>();
    lane_end_x_ = vis["lane_range"]["end_x"].as<double>();
    lane_step_ = vis["lane_range"]["step"].as<double>();
    lane_gap_ = vis["lane_range"]["gap"].as<double>();
}


void Visualizer::update_sampled_path(std::vector<std::vector<std::vector<double>>> sampled_states) {
    if (sampled_states.empty() || sampled_states[0].empty() || sampled_states[0][0].size() < 2) {
        ROS_WARN("Visualizer::update_sampled_path - empty or malformed input, skipping.");
        return;
    }

    int K = sampled_states.size();         // 샘플 개수
    int N = sampled_states[0].size();      // 각 샘플의 경로 길이

    for (int k = 0; k < K; ++k) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "sampled_paths";
        marker.id = k;  // sample index로 ID 설정
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        // 🔹 더 얇게
        marker.scale.x = 0.008;
        marker.scale.y = 0.008;
        marker.scale.z = 0.008;

        // 🔹 색상 통일 (하늘색 계열)
        marker.color.r = 0.0;
        marker.color.g = 0.7;
        marker.color.b = 1.0;
        marker.color.a = 0.6;

        // 궤적 포인트 추가
        for (int n = 0; n < N; ++n) {
            geometry_msgs::Point p;
            p.x = sampled_states[k][n][0];
            p.y = sampled_states[k][n][1];
            p.z = 0.05;

            if (!std::isfinite(p.x) || !std::isfinite(p.y))
                continue;

            marker.points.push_back(p);
        }

        if (!marker.points.empty()) {
            sampled_path_pub_.publish(marker);
        }
    }

}



int Visualizer::get_sample_number() const {return sample_number_;}