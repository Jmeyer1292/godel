#include <godel_process_execution/keyence_process_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motoman_keyence_process_service_node");

  ros::NodeHandle nh;

  std::string real_name, sim_name;

  nh.param<std::string>("actual_execution_service", real_name, "execute_kinematic_path");
  nh.param<std::string>("simulated_execution_service", sim_name, "simulation/simulate");

  godel_process_execution::KeyenceProcessExecutionService process_executor("scan_process_execution", sim_name, real_name, nh);

  ros::spin();
  return 0;
}