#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

// descartes
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "common_utils.h"


namespace godel_process_planning
{

/**
 * @brief toDescartesPt
 * @param pose
 * @return
 */
static inline descartes_core::TrajectoryPtPtr toDescartesPt(const Eigen::Affine3d& pose)
{
  using namespace descartes_trajectory;
  using namespace descartes_core;
  const static descartes_core::TimingConstraint tm (0.15);
  return TrajectoryPtPtr(new AxialSymmetricPt(pose, M_PI/6.0, AxialSymmetricPt::Z_AXIS, tm)); // default timing
}

/**
 * @brief toDescartesTraj
 * @param ref
 * @param points
 * @return
 */
static godel_process_planning::DescartesTraj toDescartesTraj(const geometry_msgs::Pose& ref,
                                                             const std::vector<geometry_msgs::Point>& points)
{
  DescartesTraj traj;
  traj.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    traj.push_back( toDescartesPt(createNominalTransform(ref, points[i])) );
  }

  return traj;
}

bool ProcessPlanningManager::handleBlendPlanning(godel_msgs::BlendProcessPlanning::Request& req,
                                                 godel_msgs::BlendProcessPlanning::Response& res)
{
  // Transform process path from geometry msgs to descartes points
  DescartesTraj process_points = toDescartesTraj(req.path.reference, req.path.points);
  DescartesTraj solved_path;
  // Capture the current state of the robot
  std::vector<double> current_joints = getCurrentJointState("joint_states");

  // Current pose
  Eigen::Affine3d init_pose;
  blend_model_->getFK(current_joints, init_pose);
  // First process point pose
  Eigen::Affine3d process_start_pose;
  process_points.front()->getNominalCartPose(std::vector<double>(), *blend_model_, process_start_pose);
  // Last process point pose
  Eigen::Affine3d process_stop_pose;
  process_points.back()->getNominalCartPose(std::vector<double>(), *blend_model_, process_stop_pose);

  // Create interpolation segment from init position to process path
  DescartesTraj to_process = createLinearPath(init_pose, process_start_pose);
  to_process.front() = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(current_joints));
  // Create interpolation segment from end of process path to init position
  DescartesTraj from_process = createLinearPath(process_stop_pose, init_pose);
  from_process.back() = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(current_joints));

  DescartesTraj seed_path;
  seed_path.insert(seed_path.end(), to_process.begin(), to_process.end());
  seed_path.insert(seed_path.end(), process_points.begin(), process_points.end());
  seed_path.insert(seed_path.end(), from_process.begin(), from_process.end());

  // Attempt to solve the initial path
  if (!descartesSolve(seed_path, blend_model_, solved_path))
  {
    return false;
  }

  // Joints

  trajectory_msgs::JointTrajectory approach = planFreeMove(*blend_model_, "manipulator_tcp", moveit_model_,
                                                           extractJoints(*blend_model_, *solved_path[0]),
                                                           extractJoints(*blend_model_, *solved_path[to_process.size()]));

  trajectory_msgs::JointTrajectory depart = planFreeMove(*blend_model_, "manipulator_tcp", moveit_model_,
                                                         extractJoints(*blend_model_, *solved_path[to_process.size() + process_points.size() -1]),
                                                         extractJoints(*blend_model_, *solved_path[seed_path.size() - 1]));

  DescartesTraj process_part (solved_path.begin() + to_process.size(), solved_path.end() - from_process.size());

  trajectory_msgs::JointTrajectory process = toROSTrajectory(process_part, *blend_model_);

  // Translate the Descartes trajectory into a ROS joint trajectory
  const std::vector< std::string >& joint_names =
      moveit_model_->getJointModelGroup("manipulator_tcp")->getActiveJointModelNames();

  res.plan.trajectory_process = process;
  res.plan.trajectory_approach = approach;
  res.plan.trajectory_depart = depart;

  godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_approach);
  godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_depart);
  godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_process);
  // set the type to blend plan
  res.plan.type = godel_msgs::ProcessPlan::BLEND_TYPE;

  return true;
}

}
