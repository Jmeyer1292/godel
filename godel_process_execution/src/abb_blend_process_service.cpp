#include <godel_process_execution/abb_blend_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include "process_utils.h"

godel_process_execution::AbbBlendProcessExecutionService::AbbBlendProcessExecutionService(const std::string& name, 
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  ROS_INFO_STREAM("Starting process execution service with name " << name << ". Using simulation service '" 
    << sim_name << "' and actual execution service: '" << real_name << "'.");

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<AbbBlendProcessExecutionService,
                                godel_msgs::BlendProcessExecution::Request,
                                godel_msgs::BlendProcessExecution::Response>
            (name, &godel_process_execution::AbbBlendProcessExecutionService::executionCallback, this);
}

bool godel_process_execution::AbbBlendProcessExecutionService::executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                                                                         godel_msgs::BlendProcessExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;

  if (req.simulate)
  {
    trajectory_msgs::JointTrajectory aggregate_traj;
    aggregate_traj = req.trajectory_approach;
    appendTrajectory(aggregate_traj, req.trajectory_process);
    appendTrajectory(aggregate_traj, req.trajectory_depart);
    // Pass the trajectory to the simulation service
    SimulateTrajectory srv;
    srv.request.wait_for_execution = false;
    srv.request.trajectory = aggregate_traj;

    if (!sim_client_.call(srv))
    {
      // currently no response fields in the simulate header
      return false;
    }
    return true;
  }
  else
  {
    //ABB Rapid Emmiter
  }


  return false;
}
