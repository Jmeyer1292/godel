#include "godel_path_planning/trajectory_utils.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


// Translates descartes trajectory points to ROS trajectory Ppoints
void godel_path_planning::populateTrajectoryMsg(const TrajectoryVec& solution,
                                                const descartes_core::RobotModel& robot_model,
                                                trajectory_msgs::JointTrajectory& trajectory)
{
  typedef std::vector<descartes_core::TrajectoryPtPtr>::const_iterator JointSolutionIterator;
  const static double DEFAULT_TIME = 0.05;
  
  ros::Duration time_from_start (0.0);
  for (JointSolutionIterator it = solution.begin(); it != solution.end(); ++it)
  {
    // Retrieve actual target joint angles from the polymorphic interface function
    std::vector<double> sol;
    it->get()->getNominalJointPose(std::vector<double>(), robot_model, sol);
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = sol;
    point.velocities.resize(sol.size(), 0.0); // Fill extra fields with zeroes for now
    point.accelerations.resize(sol.size(), 0.0);
    point.effort.resize(sol.size(), 0.0);

    double time_step = it->get()->getTiming().upper;
    if (time_step == 0.0)
    {
      time_from_start += ros::Duration(DEFAULT_TIME);
    }
    else
    {
      time_from_start += ros::Duration(time_step);
    }

    point.time_from_start = time_from_start;

    // add trajectory point to array
    trajectory.points.push_back(point);
  }
  return;
}

void godel_path_planning::populateProcessTrajectories(const TrajectoryVec& entire_path,
                                                      const descartes_core::RobotModel& robot_model,
                                                      const std::string& frame_id,
                                                      const std::vector<std::string>& joint_names,
                                                      size_t blend_start_idx,
                                                      size_t blend_stop_idx,
                                                      trajectory_msgs::JointTrajectory& approach,
                                                      trajectory_msgs::JointTrajectory& process,
                                                      trajectory_msgs::JointTrajectory& depart)
{
  approach.header.frame_id = frame_id;
  process.header.frame_id = frame_id;
  depart.header.frame_id = frame_id;

  approach.header.stamp = ros::Time::now();
  process.header.stamp = ros::Time::now();
  depart.header.stamp = ros::Time::now();

  approach.joint_names = joint_names;
  process.joint_names = joint_names;
  depart.joint_names = joint_names;

  TrajectoryVec segment;
  
  segment.assign(entire_path.begin(), entire_path.begin() + blend_start_idx);
  populateTrajectoryMsg(segment, robot_model, approach);

  segment.assign(entire_path.begin() + blend_start_idx, entire_path.begin() + blend_stop_idx);
  populateTrajectoryMsg(segment, robot_model, process);

  segment.assign(entire_path.begin() + blend_stop_idx, entire_path.end());
  populateTrajectoryMsg(segment, robot_model, depart);
}

void godel_path_planning::moveitSmoothTrajectory(trajectory_msgs::JointTrajectory& traj, const moveit::core::RobotModelConstPtr& model)
{
  using namespace moveit;

  ROS_INFO_STREAM("smoothing trajectory");

  robot_trajectory::RobotTrajectory rt(model, "manipulator");
  rt.setRobotTrajectoryMsg(moveit::core::RobotState(model), traj);
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt);

  if (!success) ROS_ERROR_STREAM("Could not time parameterize");

  moveit_msgs::RobotTrajectory moveit_rob_traj;
  rt.getRobotTrajectoryMsg(moveit_rob_traj);

  traj = moveit_rob_traj.joint_trajectory;

  const double time_ratio = 2.0; 
  auto f = [time_ratio](trajectory_msgs::JointTrajectoryPoint& pt) {
    pt.time_from_start *= time_ratio;
  };

  std::for_each(traj.points.begin(),traj.points.end(), f);


}