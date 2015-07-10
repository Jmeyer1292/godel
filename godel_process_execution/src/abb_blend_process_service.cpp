#include <godel_process_execution/abb_blend_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <fstream>

#include "process_utils.h"
#include "rapid_generator/rapid_emitter.h"
#include "abb_file_suite/ExecuteProgram.h"

// hack
#include "sensor_msgs/JointState.h"
#include "ros/topic.h"

static inline bool compare(const std::vector<double>& a, const std::vector<double>& b, double eps = 0.01)
{
  if (a.size() != b.size()) {
    ROS_WARN("Joint configs not the same size");
    return false;
  }
  double diff = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) diff += std::abs(a[i] - b[i]);

  return diff < eps;
} 

static bool waitForExecution(const std::vector<double>& end_goal, const ros::Duration& wait_for, const ros::Duration& time_out)
{
  sensor_msgs::JointStateConstPtr state; 
  ros::Time end_time = ros::Time::now() + time_out;
  // wait a fixed amount of time
  wait_for.sleep();
  ROS_INFO_STREAM("Checking joint positions");


  while (ros::Time::now() < end_time)
  {
    state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(0.0));
    if (!state)
    {
      ROS_WARN("Could not get a joint_state in time");
      return false;
    }
    if (compare(state->position, end_goal))
    {
      ROS_INFO("Goal in tolerance. Returning control.");
      return true;
    }
  }
  return false;
}

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

  real_client_ = nh.serviceClient<abb_file_suite::ExecuteProgram>("execute_program");

  nh.param<bool>("J23_coupled", j23_coupled_, false);
  ROS_INFO("ABB Blend Process Execution: Coupled joints = %d", int(j23_coupled_));
}

bool godel_process_execution::AbbBlendProcessExecutionService::executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                                                                                 godel_msgs::BlendProcessExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;
  
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = req.trajectory_approach;
  appendTrajectory(aggregate_traj, req.trajectory_process);
  appendTrajectory(aggregate_traj, req.trajectory_depart);
  // Pass the trajectory to the simulation service

  if (req.simulate)
  {
    SimulateTrajectory srv;
    srv.request.wait_for_execution = req.wait_for_execution;
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
    std::vector<rapid_emitter::TrajectoryPt> pts;
    for (std::size_t i = 0; i < aggregate_traj.points.size(); ++i)
    {
      std::vector<double> values;
      for (std::size_t j = 0; j < aggregate_traj.points[i].positions.size(); ++j)
      {
        values.push_back(aggregate_traj.points[i].positions[j] * 180.0 / M_PI);
      }
      double duration = 0.0;
      if (i > 0) 
      {
        duration = (aggregate_traj.points[i].time_from_start - aggregate_traj.points[i-1].time_from_start).toSec();
      }
      rapid_emitter::TrajectoryPt rapid_point (values, duration);
      pts.push_back(rapid_point);
    }

    // Correct for 2400 linkage
    if (j23_coupled_)
    {
      for (std::size_t i = 0; i < pts.size(); ++i)
      {
        pts[i].positions_[2] += pts[i].positions_[1];
      }
    }

    rapid_emitter::ProcessParams params;
    params.spindle_speed = 1.0;
    params.tcp_speed = 200;
    params.wolf = false;
    params.slide_force = 0.0;
    params.output_name = "do_PIO_8"; 
    
    std::ofstream fp ("/tmp/blend.mod");
    if (!fp) ROS_ERROR("PROCESS_PATH_PLANNING: Could not open file");

    rapid_emitter::emitRapidFile(fp, pts, 
      req.trajectory_approach.points.size(), 
      req.trajectory_approach.points.size() + req.trajectory_process.points.size(), 
      params);

    abb_file_suite::ExecuteProgram srv;
    srv.request.file_path = "/tmp/blend.mod";

    fp.flush();
    fp.close();

    if (!real_client_.call(srv))
    {
      ROS_WARN_STREAM("Was not able to FTP new robot path to the controller.");
      return false;
    }

    if (!req.wait_for_execution) return true;
    // If we must wait for execution, let's listen 
    return waitForExecution(req.trajectory_approach.points.front().positions, ros::Duration(5.0), ros::Duration(60.0));
  }
}
