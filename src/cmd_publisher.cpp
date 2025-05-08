// cmd_publisher_with_dwa.cpp

#include "cmd_publisher.h"
#include "dynamic_window_approach.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher"), goal_x(0.0), goal_y(0.0), goal_received(false)
{
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // sub_octomap = this->create_subscription<OctomapMsg>("octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&CmdPublisher::scan_callback, this, _1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    timer_tf = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(100ms, std::bind(&CmdPublisher::timer_cmd_callback, this));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 10, std::bind(&CmdPublisher::goal_callback, this, _1));
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization/marker", 10);
    robot = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "robot/markerArray", 10);
    all = this->create_publisher<visualization_msgs::msg::Marker>(
        "alltrajectory/marker", 10);
    currentObstacle = this->create_publisher<visualization_msgs::msg::Marker>(
        "currentObstacle/marker", 10);

    dwa_ = std::make_shared<DynamicWindowApproach>(0.0, 0.0); // 초기 목표는 임시값
    this->declare_parameter("obsTime_sec", 2.0);
    this->get_parameter("obsTime_sec", obsTime_sec);
}

void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    goal_received = true;
    dwa_->setGoal(goal_x, goal_y);
    dwa_->setObsTime(obsTime_sec);
    RCLCPP_INFO(this->get_logger(), "Received goal: x = %f, y = %f", goal_x, goal_y);
}

void CmdPublisher::timer_tf_callback()
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform map to base_scan");
        return;
    }

    x = t.transform.translation.x;
    y = t.transform.translation.y;
    z = t.transform.translation.z;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    visualization_msgs::msg::MarkerArray marker_array;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.11;
    marker.scale.y = 0.11;
    marker.scale.z = 0.11;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
    robot->publish(marker_array);

    position_updated = true;

    dwa_->setCurrentState(x, y, yaw, 0.0, 0.0); // 속도는 추후 보완 가능
}
void CmdPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    octomap::point3d world_min(-10, -10, 0);
    octomap::point3d world_max(10, 10, 2);
    // map.update(msg, world_min, world_max);

    std::vector<Eigen::Vector2d> obs_list;
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        double r = msg->ranges[i];
        if (std::isfinite(r) && r > msg->range_min && r < msg->range_max)
        {
            // 로봇 기준(x, y) 상대 좌표
            double obs_x_local = r * std::cos(angle);
            double obs_y_local = r * std::sin(angle);

            // 로봇의 위치(x, y)와 방향(yaw)을 기준으로 map 좌표계로 변환
            double obs_x_global = x + std::cos(yaw) * obs_x_local - std::sin(yaw) * obs_y_local;
            double obs_y_global = y + std::sin(yaw) * obs_x_local + std::cos(yaw) * obs_y_local;

            obs_list.emplace_back(obs_x_global, obs_y_global);
            // std::cout << "장애물 감지 : " << obs_x_global << " , " << obs_y_global << std::endl;
        }

        angle += msg->angle_increment;
    }

    // DWA에 장애물 전달
    dwa_->setObstacles(obs_list);
}

void CmdPublisher::timer_cmd_callback()
{
    if (!goal_received || !position_updated /*|| !map.is_updated()*/)
    {
        return;
    }

    // 최적 속도 계산
    Eigen::Vector2d control = dwa_->computeBestControl();

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = control(0);
    cmd.angular.z = control(1);
    pub_cmd->publish(cmd);

    visualizeAllTrajectories(dwa_->getAllTrajectories());
    visualizeTrajectory(dwa_->getBestTrajectory());
    visualizeObstacle(dwa_->getObstacles());

    if (std::hypot(goal_x - x, goal_y - y) < 0.2)
    {
        RCLCPP_INFO(this->get_logger(), "목표에 도달했습니다!");
        geometry_msgs::msg::Twist stop;
        pub_cmd->publish(stop);
        goal_received = false;
    }
}
void CmdPublisher::visualizeObstacle(const std::vector<Eigen::Vector2d> &Obstacle)
{
    int id = 0;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "current_obstacle";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto &obs : Obstacle)
    {
        geometry_msgs::msg::Point p;
        p.x = obs.x();
        p.y = obs.y();
        p.z = 0.0;
        marker.points.push_back(p);
        // std::cout << "저장된 장애물 : " << p.x << "," << p.y << std::endl;
    }
    currentObstacle->publish(marker);
}

void CmdPublisher::visualizeAllTrajectories(const std::vector<std::vector<Eigen::VectorXd>> &trajectories)
{
    int id = 0;
    for (const auto &traj : trajectories)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "dwa_all_trajectories";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.7;

        for (const auto &state : traj)
        {
            geometry_msgs::msg::Point pt;
            pt.x = state(0);
            pt.y = state(1);
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        // RCLCPP_INFO(this->get_logger(), "Publishing trajectory with %zu points", marker.points.size());
        all->publish(marker);
    }
}

void CmdPublisher::visualizeTrajectory(const std::vector<Eigen::VectorXd> &trajectory)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "dwa_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto &state : trajectory)
    {
        geometry_msgs::msg::Point p;
        p.x = state(0);
        p.y = state(1);
        p.z = 0.0;
        marker.points.push_back(p);
    }
    // RCLCPP_INFO(this->get_logger(), "Publishing trajectory with %zu points", marker.points.size());

    marker_pub->publish(marker);
}
void CmdPublisher::octomap_callback(const OctomapMsg &octomap_msg)
{
    octomap::point3d world_min(-10, -10, 0);
    octomap::point3d world_max(10, 10, 2);
    map.update(octomap_msg, world_min, world_max);
}