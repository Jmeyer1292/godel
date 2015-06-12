#include <godel_process_execution/keyence_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include "godel_msgs/TrajectoryExecution.h"

#include "process_utils.h"

#include <boost/thread.hpp>

#include "keyence_driver/ChangeProgram.h"

#include <ros/topic.h>

godel_process_execution::KeyenceProcessExecutionService::KeyenceProcessExecutionService(const std::string& name, 
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  ROS_INFO_STREAM("Starting process execution service with name " << name << ". Using simulation service '" 
    << sim_name << "' and actual execution service: '" << real_name << "'.");

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<KeyenceProcessExecutionService,
                                godel_msgs::KeyenceProcessExecution::Request,
                                godel_msgs::KeyenceProcessExecution::Response>
            (name, &godel_process_execution::KeyenceProcessExecutionService::executionCallback, this);

  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>("path_execution");

  keyence_client_ = nh.serviceClient<keyence_driver::ChangeProgram>("change_program");


}

bool godel_process_execution::KeyenceProcessExecutionService::executionCallback(godel_msgs::KeyenceProcessExecution::Request& req,
                                                                                godel_msgs::KeyenceProcessExecution::Response& res)
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
    boost::thread(&godel_process_execution::KeyenceProcessExecutionService::executeProcess, this, req);
    
    return true;
  }


}

void godel_process_execution::KeyenceProcessExecutionService::executeProcess(godel_msgs::KeyenceProcessExecution::Request& req)
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
    return;
  }

  // Turn on sensor
  keyence_driver::ChangeProgram keyence_srv;
  keyence_srv.request.program_no = 3;

  if (!keyence_client_.call(keyence_srv))
  {
    ROS_WARN_STREAM("Could not active keyence");
    return;
  }
  
  // Capture current robot position
  ros::Duration(1.5).sleep();
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", ros::Duration(1.0));
  if (!state) ROS_ERROR_STREAM("Could not capture joints");

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = state->position;
  std::vector<double> dummy (state->position.size(), 0.0);
  pt.velocities = dummy;
  pt.accelerations = dummy;
  pt.effort = dummy;
  pt.time_from_start = ros::Duration(0.5);
  
  trajectory_msgs::JointTrajectory fixed_traj;
  fixed_traj.joint_names = req.trajectory_process.joint_names;
  fixed_traj.header = req.trajectory_process.header;

  fixed_traj.points.push_back(pt);
  appendTrajectory(fixed_traj, req.trajectory_process);
  srv_process.request.trajectory = fixed_traj;

  if (!real_client_.call(srv_process))
  {
    // res.code = srv_process.response.error_code.val;
    return ;
  }

  // Turn keyence off
  keyence_srv.request.program_no = 0;
  if (!keyence_client_.call(keyence_srv))
  {
    ROS_WARN_STREAM("Could not de-active keyence");
    return;
  }

  if (!real_client_.call(srv_depart))
  {
    // res.code = srv_depart.response.error_code.val;
    return ;
  }
}