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
    ROS_WARN_STREAM("Sim blend");
    // Pass the trajectory to the simulation service
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
    boost::thread(&godel_process_execution::MotomanBlendProcessExecutionService::executeProcess, this, req);
    return true;
  }


  return false;
}

void godel_process_execution::MotomanBlendProcessExecutionService::executeProcess(godel_msgs::BlendProcessExecution::Request req)
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

  ////////////////////////////////////
  // Capture current robot position //
  ////////////////////////////////////
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

  //////////////
  // End hack //
  //////////////

  if (!real_client_.call(srv_process))
  {
    // res.code = srv_process.response.error_code.val;
    return ;
  }

  ////////////////////////////////////
  // Capture current robot position //
  ////////////////////////////////////
  ros::Duration(2.0).sleep();
  sensor_msgs::JointStateConstPtr state2 = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", ros::Duration(1.0));
  if (!state2) ROS_ERROR_STREAM("Could not capture joints");

  trajectory_msgs::JointTrajectoryPoint pt2;
  pt2.positions = state2->position;
  pt2.velocities = dummy;
  pt2.accelerations = dummy;
  pt2.effort = dummy;
  pt2.time_from_start = ros::Duration(0.5);
  
  trajectory_msgs::JointTrajectory fixed_traj2;
  fixed_traj2.joint_names = req.trajectory_depart.joint_names;
  fixed_traj2.header = req.trajectory_depart.header;

  fixed_traj2.points.push_back(pt2);
  appendTrajectory(fixed_traj2, req.trajectory_depart);
  srv_depart.request.trajectory = fixed_traj2;

  //////////////
  // End hack //
  //////////////

  // Capture current robot position
  // ros::Duration(1.5).sleep();
  // srv_depart.request.trajectory = insertCurrentPosition(req.trajectory_depart);

  if (!real_client_.call(srv_depart))
  {
    // res.code = srv_depart.response.error_code.val;
    return;
  }

  ROS_WARN_STREAM("Executing motoman blend trajectory (3)");
}