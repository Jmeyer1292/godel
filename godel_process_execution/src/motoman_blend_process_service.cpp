#include <godel_process_execution/motoman_blend_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <godel_msgs/TrajectoryExecution.h>

#include "process_utils.h"
#include <boost/thread.hpp>

#include <ros/topic.h>

godel_process_execution::MotomanBlendProcessExecutionService::MotomanBlendProcessExecutionService(const std::string& name, 
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  ROS_INFO_STREAM("Starting process execution service with name " << name << ". Using simulation service '" 
    << sim_name << "' and actual execution service: '" << real_name << "'.");

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<MotomanBlendProcessExecutionService,
                                godel_msgs::BlendProcessExecution::Request,
                                godel_msgs::BlendProcessExecution::Response>
            (name, &godel_process_execution::MotomanBlendProcessExecutionService::executionCallback, this);

  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>("path_execution");

}

bool godel_process_execution::MotomanBlendProcessExecutionService::executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                                                                         godel_msgs::BlendProcessExecution::Response& res)
{
  using simulator_service::SimulateTrajectory;

  if (req.simulate)
  {
    ROS_WARN_STREAM("Sim blend with wait_for_execution: " << int(req.wait_for_execution));
    // Pass the trajectory to the simulation service
    trajectory_msgs::JointTrajectory aggregate_traj;
    aggregate_traj = req.trajectory_approach;
    appendTrajectory(aggregate_traj, req.trajectory_process);
    appendTrajectory(aggregate_traj, req.trajectory_depart);
    // Pass the trajectory to the simulation service
    SimulateTrajectory srv;
    srv.request.wait_for_execution = req.wait_for_execution;
    srv.request.trajectory = aggregate_traj;
    // Call simulation service
    if (!sim_client_.call(srv))
    {
      // currently no response fields in the simulate header
      return false;
    }
    return true;
  }
  else
  {
    // Real execution; if we shouldn't wait, spawn a thread and return this function immediately
    // No way to check for success or failure currently. Need future.
    if (req.wait_for_execution)
      return executeProcess(req);
    else
      boost::thread(&godel_process_execution::MotomanBlendProcessExecutionService::executeProcess, this, req);
    return true;
  }


  return false;
}

bool godel_process_execution::MotomanBlendProcessExecutionService::executeProcess(godel_msgs::BlendProcessExecution::Request req)
{
  godel_msgs::TrajectoryExecution srv_approach;
  srv_approach.request.wait_for_execution = true;
  srv_approach.request.trajectory = req.trajectory_approach;

  godel_msgs::TrajectoryExecution srv_process;
  srv_process.request.wait_for_execution = true;
  srv_process.request.trajectory = req.trajectory_process;

  godel_msgs::TrajectoryExecution srv_depart;
  srv_depart.request.wait_for_execution = true;
  srv_depart.request.trajectory = req.trajectory_depart;

  if (!real_client_.call(srv_approach))
  {
    // res.code = srv_approach.response.error_code.val;
    return false;
  }

  srv_process.request.trajectory = req.trajectory_process;

  if (!real_client_.call(srv_process))
  {
    // res.code = srv_process.response.error_code.val;
    return false;
  }

  srv_depart.request.trajectory = req.trajectory_depart;

  if (!real_client_.call(srv_depart))
  {
    // res.code = srv_depart.response.error_code.val;
    return false;
  }
  return true;
}