/**
 * Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
 * @file SrBenchmarkExecutor.cpp
 * @author Michal Kramarczyk <michal@shadowrobot.com>
 * @date Nov 01 2018
 **/

#include <sr_moveit_planner_benchmarking/SrBenchmarkExecutor.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>

#include <vector>
#include <string>

namespace sr_moveit_planner_benchmarking
{
SrBenchmarkExecutor::SrBenchmarkExecutor(const std::string& robot_description_param)
    : moveit_ros_benchmarks::BenchmarkExecutor(robot_description_param)
{
}

SrBenchmarkExecutor::~SrBenchmarkExecutor()
{
}

void SrBenchmarkExecutor::collectMetrics(PlannerRunData& metrics,
                                       const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved,
                                       double total_time)
{
    if (solved)
    {
        double plan_quality = 0.0;
        double plan_quality_cart = 0.0;
        moveit_ros_benchmarks::BenchmarkExecutor::collectMetrics(metrics, mp_res, solved, total_time);
        for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
        {
            const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];
            plan_quality = evaluate_plan(p);
            plan_quality_cart = evaluate_plan_cart(p);
            metrics["path_" + mp_res.description_[j] + "_plan_quality REAL"] =
                boost::lexical_cast<std::string>(plan_quality);
            metrics["path_" + mp_res.description_[j] + "_plan_quality_cartesian REAL"] =
                boost::lexical_cast<std::string>(plan_quality_cart);
        }
    }
}

double evaluate_plan(const robot_trajectory::RobotTrajectory& p)
{
    int num_of_joints = p.getWayPoint(0).getVariableCount();

    std::vector<int> weights(num_of_joints, 0);
    for (int k = 0; k < num_of_joints; k++)
    {
        weights[k] = num_of_joints - k;
    }

    std::vector<std::vector <double> > plan_array(p.getWayPointCount(), std::vector<double>(num_of_joints));
    for (size_t i = 0 ; i < p.getWayPointCount() ; ++i)
    {
        for (size_t j = 0 ; j < num_of_joints ; ++j)
        {
        plan_array[i][j] = p.getWayPoint(i).getVariablePositions()[j];
        }
    }

    std::vector<std::vector <double> > deltas(p.getWayPointCount()-1, std::vector<double>(num_of_joints));
    for (size_t i = 0 ; i < p.getWayPointCount()-1 ; ++i)
    {
        for (size_t j = 0 ; j < num_of_joints ; ++j)
        {
        deltas[i][j] = plan_array[i+1][j] - plan_array[i][j];
        if (deltas[i][j] < 0)
        {
            deltas[i][j] = - deltas[i][j];
        }
        }
    }

    std::vector<double> sum_deltas(num_of_joints, 0);
    for (size_t i = 0 ; i < p.getWayPointCount()-1 ; ++i)
    {
        for (size_t j = 0 ; j < num_of_joints ; ++j)
        {
        sum_deltas[j] += deltas[i][j];
        }
    }

    std::vector<double> sum_deltas_weighted(num_of_joints, 0);
    for (size_t j = 0 ; j < num_of_joints ; ++j)
    {
        sum_deltas_weighted[j] = sum_deltas[j] * weights[j];
    }

    double plan_quality = 0.0;
    for (auto it = sum_deltas_weighted.begin() ; it != sum_deltas_weighted.end(); ++it)
    {
        plan_quality += *it;
    }
    return plan_quality;
}

double evaluate_plan_cart(const robot_trajectory::RobotTrajectory& p)
{
    std::vector<geometry_msgs::Transform> transforms(p.getWayPointCount());
    for (size_t i = 0 ; i < p.getWayPointCount(); ++i)
    {
        moveit::core::RobotState goal_state = p.getWayPoint(i);
        const moveit::core::JointModel* joint_eef = goal_state.getJointModel(goal_state.getVariableNames()
            [goal_state.getVariableCount()-1]);
        std::string link_eef  = joint_eef->getChildLinkModel()->getName();
        const Eigen::Affine3d& link_pose = goal_state.getGlobalLinkTransform(link_eef);
        tf::transformEigenToMsg(link_pose, transforms[i]);
    }

    int n = p.getWayPointCount()-1;
    double eef_dist = 0.0;
    double eef_rot  = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        geometry_msgs::Quaternion q2 = transforms[i+1].rotation;
        geometry_msgs::Quaternion q1 = transforms[i  ].rotation;
        double x, y, z, w;
        x = fabs(transforms[i+1].translation.x - transforms[i].translation.x);
        y = fabs(transforms[i+1].translation.y - transforms[i].translation.y);
        z = fabs(transforms[i+1].translation.z - transforms[i].translation.z);
        w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        eef_dist += sqrt(x*x+y*y+z*z);
        eef_rot += 2*acos(w);
    }

    geometry_msgs::Quaternion qn = transforms[n].rotation;
    geometry_msgs::Quaternion q0 = transforms[0].rotation;
    double x_t, y_t, z_t, w_t;
    double tot_dist, tot_rot;
    x_t = fabs(transforms[n].translation.x - transforms[0].translation.x);
    y_t = fabs(transforms[n].translation.y - transforms[0].translation.y);
    z_t = fabs(transforms[n].translation.z - transforms[0].translation.z);
    w_t = -qn.x * q0.x - qn.y * q0.y - qn.z * q0.z + qn.w * q0.w;
    tot_dist = sqrt(x_t*x_t+y_t*y_t+z_t*z_t);
    tot_rot = 2*acos(w_t);

    double plan_quality = 0.0;
    if (tot_dist > 0.001)
        plan_quality += eef_dist/tot_dist;
    else
        plan_quality += 1;
    if (tot_rot  > 0.001)
        plan_quality += eef_rot/tot_rot;
    else
        plan_quality += 1;

    plan_quality /= 2;

    return plan_quality;
}
}  // namespace sr_moveit_planner_benchmarking
