/*#ifndef ROS2_TERM_PROJECT_CMD_PUBLISHER_H
#define ROS2_TERM_PROJECT_CMD_PUBLISHER_H*/
#pragma once

#include "dynamic_window_approach.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "map.h"
#include "octomap/OcTree.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <queue>
#include <rclcpp/publisher.hpp>
#include <set>
#include <vector>

class CmdPublisher : public rclcpp::Node
{
public:
    CmdPublisher();

private:
    void timer_tf_callback();
    void timer_cmd_callback();
    void octomap_callback(const OctomapMsg &octomap_msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void visualizeTrajectory(const std::vector<Eigen::VectorXd> &trajectory);

    double x, y, z;
    double yaw;
    double goal_x = 0.0;
    double goal_y = 0.0;
    bool position_updated;
    bool goal_received = false;
    int trajectory_marker_id = 0;
    rclcpp::Time prev_time;

    Map map;

    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    std::shared_ptr<DynamicWindowApproach> dwa_;

    std::vector<geometry_msgs::msg::Point> path_history_; // 로봇 이동 기록
};

// #endif // ROS2_TERM_PROJECT_CMD_PUBLISHER_H
