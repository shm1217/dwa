// dynamic_window_approach.cpp

#include "dynamic_window_approach.h"
#include "rclcpp/rclcpp.hpp"

DynamicWindowApproach::DynamicWindowApproach(double goal_x, double goal_y)
{
    currentState_ = Eigen::VectorXd::Zero(5); // x, y, theta, v, w 초기화
    goal_ = Eigen::Vector2d(goal_x, goal_y);
    dynamic_window_.resize(4); // dynamic window 크기 설정
    /*marker_tra = this->create_publisher<visualization_msgs::msg::Marker>(
        "trajectory/marker", 10);*/
}

void DynamicWindowApproach::setCurrentState(double x, double y, double theta, double v, double w)
{
    currentState_ << x, y, theta, v, w;
}

void DynamicWindowApproach::setGoal(double x, double y)
{
    goal_ << x, y;
}

void DynamicWindowApproach::setObstacles(const std::vector<Eigen::Vector2d> &obs)
{
    rclcpp::Time now = rclcpp::Clock().now();
    for (const auto& o : obs)
    {
        timed_obstacles_.push_back(std::make_tuple(now, o.x(), o.y()));
    }

    // 오래된 장애물 제거 (2초 기준)
    auto it = timed_obstacles_.begin();
    while (it != timed_obstacles_.end())
    {
        if ((now - std::get<0>(*it)).seconds() > 2.0)
        {
            it = timed_obstacles_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // 업데이트된 장애물 벡터 생성
    std::vector<Eigen::Vector2d> filtered_obs;
    for (const auto& item : timed_obstacles_)
    {
        filtered_obs.emplace_back(std::get<1>(item), std::get<2>(item));
    }

    obstacles_ = filtered_obs;
}

const std::vector<Eigen::Vector2d> &DynamicWindowApproach::getObstacles() const
{
    return obstacles_;
}


Eigen::Vector2d DynamicWindowApproach::computeBestControl() // (v,w) 계산하고 최적의 조합 반환, 최종적으로 사용하는 함수
{
    computeDynamicWindow();
    double min_cost = std::numeric_limits<double>::max();
    Eigen::Vector2d best_control(0.0, 0.0);
    best_trajectory_.clear();
    all_trajectories_.clear();

    for (double v = dynamic_window_[0]; v <= dynamic_window_[1]; v += v_res_)
    {
        for (double w = dynamic_window_[2]; w <= dynamic_window_[3]; w += w_res_)
        {
            Eigen::Vector2d control(v, w);
            auto traj = simulateTrajectory(control);
            all_trajectories_.push_back(traj); // 후보 저장

            double cost = evaluateTrajectoryCost(traj);
            if (cost < min_cost)
            {
                min_cost = cost;
                best_control = control;
                best_trajectory_ = traj;
            }
        }
    }

    return best_control;
}
const std::vector<std::vector<Eigen::VectorXd>> &DynamicWindowApproach::getAllTrajectories() const
{
    return all_trajectories_;
}

std::vector<Eigen::VectorXd> DynamicWindowApproach::simulateTrajectory(const Eigen::Vector2d &control) // 경로 저장
{
    std::vector<Eigen::VectorXd> trajectory;
    Eigen::VectorXd state = currentState_;
    double time = 0.0;
    trajectory.push_back(state);

    while (time <= predict_time_)
    {
        state(2) += control(1) * dt_;                      // state(2)=로봇의 방향
        state(0) += control(0) * std::cos(state(2)) * dt_; // state(0)=로봇의 현재 x좌표
        state(1) += control(0) * std::sin(state(2)) * dt_; // state(1)=로봇의 현재 y좌표
        state(3) = control(0);                             // state(3) 로봇의 선속도
        state(4) = control(1);                             // state(4)=로봇의 각속도
        trajectory.push_back(state);
        time += dt_;
    }
    return trajectory;
}

void DynamicWindowApproach::computeDynamicWindow() // 로봇이 이동 가능한 속도 범위 설정
{
    double v = currentState_(3);
    double w = currentState_(4);

    dynamic_window_[0] = std::max(min_speed_, v - max_accel_ * dt_);
    dynamic_window_[1] = std::min(max_speed_, v + max_accel_ * dt_);
    dynamic_window_[2] = std::max(-max_yawrate_, w - max_dyawrate_ * dt_);
    dynamic_window_[3] = std::min(max_yawrate_, w + max_dyawrate_ * dt_);
}

std::vector<Eigen::VectorXd> DynamicWindowApproach::getBestTrajectory() const
{
    return best_trajectory_;
}

double DynamicWindowApproach::evaluateTrajectoryCost(const std::vector<Eigen::VectorXd> &trajectory) // 최종 목적함수
{
    return weight_goal_ * goalCost(trajectory) + weight_obstacle_ * obstacleCost(trajectory) + weight_velocity_ * velocityCost(trajectory);
}

double DynamicWindowApproach::goalCost(const std::vector<Eigen::VectorXd> &trajectory) // heading, 로봇이 목적지로 잘 향하도록
{
    Eigen::VectorXd last = trajectory.back();
    return (goal_ - last.head<2>()).norm();
}

double DynamicWindowApproach::obstacleCost(const std::vector<Eigen::VectorXd> &trajectory) // clearance, 로봇이 장애물에 가까이 가지 않도록
{

    double min_dist = std::numeric_limits<double>::max();
    for (const auto &state : trajectory)
    {
        for (const auto &obs : obstacles_)
        {
            // std::cout << "장애물 위치: x = " << obs.x() << ", y = " << obs.y() << std::endl;
            double dist = (obs - state.head<2>()).norm();
            if (dist < robot_radius_)
                return 1e6; // 충돌
            if (dist < min_dist)
                min_dist = dist;
        }
    }
    return 1.0 / min_dist;
}

double DynamicWindowApproach::velocityCost(const std::vector<Eigen::VectorXd> &trajectory) // velocity, 로봇이 빠른 속도로 움직이도록
{
    return max_speed_ - trajectory.back()(3);
}
