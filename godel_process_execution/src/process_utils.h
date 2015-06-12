#ifndef PATH_GODEL_PROCESS_UTILS_H
#define PATH_GODEL_PROCESS_UTILS_H

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace godel_process_execution
{

void appendTrajectory(trajectory_msgs::JointTrajectory& original, 
                      const trajectory_msgs::JointTrajectory& next);

inline
trajectory_msgs::JointTrajectory insertCurrentPosition(const trajectory_msgs::JointTrajectory traj)
{
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
  fixed_traj.joint_names = traj.joint_names;
  //fixed_traj.header = traj.header;

  fixed_traj.points.push_back(pt);
  appendTrajectory(fixed_traj, traj);
}


}

#endif
