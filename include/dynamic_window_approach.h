// dynamic_window_approach.hpp

#pragma once

#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <rclcpp/publisher.hpp>
#include <set>
#include <vector>

class DynamicWindowApproach //: public rclcpp::Node
{
public:
    DynamicWindowApproach(double goal_x, double goal_y);

    void setCurrentState(double x, double y, double theta, double v, double w);
    void setGoal(double x, double y);
    void setObstacles(const std::vector<Eigen::Vector2d> &obs);

    Eigen::Vector2d computeBestControl();
    std::vector<Eigen::VectorXd> getBestTrajectory() const;
    const std::vector<std::vector<Eigen::VectorXd>> &getAllTrajectories() const;
    const std::vector<Eigen::Vector2d> &getObstacles() const;


private:
    void computeDynamicWindow();
    std::vector<Eigen::VectorXd> simulateTrajectory(const Eigen::Vector2d &control);
    double evaluateTrajectoryCost(const std::vector<Eigen::VectorXd> &trajectory);

    double goalCost(const std::vector<Eigen::VectorXd> &trajectory);
    double obstacleCost(const std::vector<Eigen::VectorXd> &trajectory);
    double velocityCost(const std::vector<Eigen::VectorXd> &trajectory);

private:
    Eigen::VectorXd currentState_; // x, y, theta, v, w
    Eigen::Vector2d goal_;
    std::vector<Eigen::Vector2d> obstacles_;
    std::vector<std::tuple<rclcpp::Time, double, double>> timed_obstacles_;

    std::vector<double> dynamic_window_; // [v_min, v_max, w_min, w_max]
    std::vector<Eigen::VectorXd> best_trajectory_;
    std::vector<std::vector<Eigen::VectorXd>> all_trajectories_;

    double dt_ = 0.1;           // 0.1
    double predict_time_ = 3.0; // 2.0

    // robot limits
    double max_speed_ = 0.4;
    double min_speed_ = 0.0;
    double max_yawrate_ = 1.5;
    double max_accel_ = 0.4;
    double max_dyawrate_ = 2.0;

    // resolutions
    double v_res_ = 0.02;
    double w_res_ = 0.1;

    // robot size
    double robot_radius_ = 0.2;

    // cost weights
    double weight_goal_ = 1.0;
    double weight_obstacle_ = 0.3;
    double weight_velocity_ = 2.0;
};
