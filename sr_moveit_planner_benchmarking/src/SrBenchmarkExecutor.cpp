#include <sr_moveit_planner_benchmarking/SrBenchmarkExecutor.h>

namespace sr_moveit_planner_benchmarking
{
SrBenchmarkExecutor::SrBenchmarkExecutor(const std::string& robot_description_param) : moveit_ros_benchmarks::BenchmarkExecutor(robot_description_param)
{
}

SrBenchmarkExecutor::~SrBenchmarkExecutor()
{
}

void SrBenchmarkExecutor::collectMetrics(PlannerRunData& metrics,
                                       const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved,
                                       double total_time)
{
    ROS_INFO_STREAM("******** Calling child executor **********");
    moveit_ros_benchmarks::BenchmarkExecutor::collectMetrics(metrics, mp_res, solved, total_time);
}
}