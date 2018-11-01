/**
 * Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
 * @file SrBenchmarkExecutor.h
 * @author Michal Kramarczyk <michal@shadowrobot.com>
 * @date Nov 01 2018
 **/

#ifndef SR_MOVEIT_PLANNER_BENCHMARKING_SR_BENCHMARK_EXECUTOR_H
#define SR_MOVEIT_PLANNER_BENCHMARKING_SR_BENCHMARK_EXECUTOR_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <moveit/benchmarks/BenchmarkExecutor.h>

namespace sr_moveit_planner_benchmarking
{
class SrBenchmarkExecutor : public moveit_ros_benchmarks::BenchmarkExecutor
{
    public:
        explicit SrBenchmarkExecutor(const std::string& robot_description_param = "robot_description");
        ~SrBenchmarkExecutor();

    protected:
        void collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res,
                            bool solved, double total_time);
};

double evaluate_plan(const robot_trajectory::RobotTrajectory& p);
double evaluate_plan_cart(const robot_trajectory::RobotTrajectory& p);
}  //  namespace sr_moveit_planner_benchmarking

#endif //  SR_MOVEIT_PLANNER_BENCHMARKING_SRBENCHMARKEXECUTOR_H
