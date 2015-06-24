#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <descartes_core/trajectory_pt.h>
#include <descartes_core/robot_model.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

namespace godel_process_planning
{
  typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTraj;

/**
   * @brief createNominalTransform
   * @param ref_pose
   * @param pt
   * @return
   */
  Eigen::Affine3d createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                         const geometry_msgs::Point& pt);

  /**
   * @brief descartesSolve
   * @param in_path
   * @param robot_model
   * @param out_path
   * @return
   */
  bool descartesSolve(const DescartesTraj& in_path,
                      descartes_core::RobotModelConstPtr robot_model,
                      DescartesTraj& out_path);
  /**
   * @brief toROSTrajectory
   * @param solution
   * @param model
   * @return
   */
  trajectory_msgs::JointTrajectory toROSTrajectory(const DescartesTraj& solution,
                                                   const descartes_core::RobotModel& model);
  /**
   * @brief fillTrajectoryHeaders
   * @param joints
   * @param traj
   */
  void fillTrajectoryHeaders(const std::vector<std::string>& joints,
                             trajectory_msgs::JointTrajectory& traj);

  /**
   * @brief getCurrentJointState
   * @param topic
   * @return
   */
  std::vector<double> getCurrentJointState(const std::string& topic);

  /**
   * @brief createLinearPath
   * @param start
   * @param stop
   * @return
   */
  DescartesTraj createLinearPath(const Eigen::Affine3d& start,
                                 const Eigen::Affine3d& stop);
  /**
   * @brief createJointPath
   * @param start
   * @param stop
   * @return
   */
  DescartesTraj createJointPath(const std::vector<double>& start,
                                const std::vector<double>& stop);
  /**
   * @brief getMoveitPlan
   * @param group_name
   * @param joints_start
   * @param joints_stop
   * @param model
   * @return
   */
  trajectory_msgs::JointTrajectory getMoveitPlan(const std::string& group_name,
                                                 const std::vector<double>& joints_start,
                                                 const std::vector<double>& joints_stop,
                                                 moveit::core::RobotModelConstPtr model);
  /**
   * @brief extractJoints
   * @param model
   * @param pt
   * @return
   */
  static inline
  std::vector<double> extractJoints(const descartes_core::RobotModel& model,
                                    const descartes_core::TrajectoryPt& pt)
  {
    std::vector<double> dummy, result;
    pt.getNominalJointPose(dummy, model, result);
    return result;
  }

  trajectory_msgs::JointTrajectory planFreeMove(descartes_core::RobotModel& model,
                                                const std::string &group_name,
                                                moveit::core::RobotModelConstPtr moveit_model,
                                                const std::vector<double>& start,
                                                const std::vector<double>& stop);

}

#endif // COMMON_UTILS_H

