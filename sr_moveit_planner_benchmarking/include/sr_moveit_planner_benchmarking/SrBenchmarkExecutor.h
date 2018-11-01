#ifndef SR_MOVEIT_PLANNER_BENCHMARKING_SR_BENCHMARK_EXECUTOR_
#define SR_MOVEIT_PLANNER_BENCHMARKING_SR_BENCHMARK_EXECUTOR_

#include <ros/ros.h>
#include <string>
#include <moveit/benchmarks/BenchmarkExecutor.h>

namespace sr_moveit_planner_benchmarking
{
class SrBenchmarkExecutor : public moveit_ros_benchmarks::BenchmarkExecutor
{
    public:
        SrBenchmarkExecutor(const std::string& robot_description_param = "robot_description");
        ~SrBenchmarkExecutor();

    protected:
        void collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res,
                            bool solved, double total_time);
};
}

#endif