#include <godel_path_execution/path_execution_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

godel_path_execution::PathExecutionService::PathExecutionService(const std::string& name, 
                                                                 const std::string& sim_name,
                                                                 const std::string& real_name,
                                                                 ros::NodeHandle& nh)
  : name_(name)
  , ac_("joint_trajectory_action", true)
{
  ROS_INFO_STREAM("Starting path execution service with name " << name << ". Using simulation service '" 
    << sim_name << "' and actual execution service: '" << real_name << "'.");
  
  real_client_ = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(real_name);

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<PathExecutionService,
                                godel_msgs::TrajectoryExecution::Request,
                                godel_msgs::TrajectoryExecution::Response>
            (name, &godel_path_execution::PathExecutionService::executionCallback, this);
}

bool godel_path_execution::PathExecutionService::executionCallback(godel_msgs::TrajectoryExecution::Request& req,
                                                                   godel_msgs::TrajectoryExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;

  ROS_INFO_STREAM("PathExecutionService '" << name_ << "': recieved new trajectory (simulate ==" << (req.simulate ? "true" : "false") << ")");

  if (req.trajectory.points.empty()) {
    ROS_WARN_STREAM("Cannot execute path with no trajectory points. Ignoring");
    return true;
  }

  if (req.simulate)
  {
    // // Pass the trajectory to the simulation service
    // SimulateTrajectory srv;
    // srv.request.wait_for_execution = req.wait_for_execution;
    // srv.request.trajectory = req.trajectory;

    // if (sim_client_.call(srv))
    // {
    //   // currently no response fields in the simulate header
    //   return true;
    // }
    // ROS_WARN_STREAM("PathExecutionService: Failed to call simulation service");
  }
  else
  {
    // Pass the trajectory to moveit
    // ExecuteKnownTrajectory srv;
    // srv.request.wait_for_execution = req.wait_for_execution;
    // srv.request.trajectory.joint_trajectory = req.trajectory;

    // if (real_client_.call(srv))
    // {
    //   res.code = srv.response.error_code.val;
    //   return true;
    // }
    // ROS_WARN_STREAM("PathExecutionService: Failed to call hardware execution service");
    if (!ac_.waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR_STREAM("Could not connect to action server");
      return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = req.trajectory;
    ROS_WARN_STREAM("Sending action server goal");
    ac_.sendGoal(goal);

    if (req.wait_for_execution && ac_.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start))
    {
      ROS_INFO_STREAM("Action server reported successful execution");
    }
    return true;
  }

  return false;
}
