/*
* Copyright 2017 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file SrBenchmarkExecutor.h
* @author Michal Kramarczyk <michal@shadowrobot.com>
* @date Nov 01 2018
*/

#ifndef SR_MOVEIT_PLANNER_BENCHMARKING_SRBENCHMARKEXECUTOR_H
#define SR_MOVEIT_PLANNER_BENCHMARKING_SRBENCHMARKEXECUTOR_H

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
}  // namespace sr_moveit_planner_benchmarking

#endif  // SR_MOVEIT_PLANNER_BENCHMARKING_SRBENCHMARKEXECUTOR_H
