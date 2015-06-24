#include "common_utils.h"
#include <eigen_conversions/eigen_msg.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <ros/topic.h>
#include "trajectory_utils.h"

Eigen::Affine3d godel_process_planning::createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                                               const geometry_msgs::Point& pt)
{
  Eigen::Affine3d eigen_pose;
  Eigen::Vector3d eigen_pt;

  tf::poseMsgToEigen(ref_pose, eigen_pose);
  tf::pointMsgToEigen(pt, eigen_pt);

  // Translation transform
  Eigen::Affine3d to_point;
  to_point = Eigen::Translation3d(eigen_pt);

  // Reverse the Z axis
  Eigen::Affine3d flip_z;
  flip_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  return eigen_pose * to_point * flip_z;
}

bool godel_process_planning::descartesSolve(const godel_process_planning::DescartesTraj& in_path,
                                            descartes_core::RobotModelConstPtr robot_model,
                                            godel_process_planning::DescartesTraj& out_path)
{
  // Create planner
  descartes_core::PathPlannerBasePtr planner (new descartes_planner::DensePlanner);
  planner->initialize(robot_model);

  // Attempt to solve the trajectory
  if (!planner->planPath(in_path))
  {
    ROS_ERROR("%s: Failed to plan for given trajectory.", __FUNCTION__);
    return false;
  }

  if (!planner->getPath(out_path))
  {
    ROS_ERROR("%s: Failed to retrieve path.", __FUNCTION__);
    return false;
  }

  return true;
}

trajectory_msgs::JointTrajectory
godel_process_planning::toROSTrajectory(const godel_process_planning::DescartesTraj& solution,
                                        const descartes_core::RobotModel& model)
{
  ros::Duration from_start(0.0);
  std::vector<double> joint_point;
  trajectory_msgs::JointTrajectory ros_trajectory; // result

  for (std::size_t i = 0; i < solution.size(); ++i)
  {
    solution[i]->getNominalJointPose(std::vector<double>(), model, joint_point);

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joint_point;
    pt.velocities.resize(joint_point.size(), 0.0);
    pt.accelerations.resize(joint_point.size(), 0.0);
    pt.effort.resize(joint_point.size(), 0.0);

    double time_step = solution[i]->getTiming().upper; // request descartes timing
    if (time_step == 0.0) from_start += ros::Duration(0.1); // default time
    else from_start += ros::Duration(time_step);

    pt.time_from_start = from_start;

    ros_trajectory.points.push_back(pt);
  }

  return ros_trajectory;
}

void godel_process_planning::fillTrajectoryHeaders(const std::vector<std::string>& joints,
                                                    trajectory_msgs::JointTrajectory& traj)
{
  traj.joint_names = joints;
  traj.header.frame_id = "world_frame";
  traj.header.stamp = ros::Time::now();
}

std::vector<double> godel_process_planning::getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}



godel_process_planning::DescartesTraj
godel_process_planning::createLinearPath(const Eigen::Affine3d &start,
                                         const Eigen::Affine3d &stop)
{
  PoseVector cart_path = interpolateCartesian(start, stop, 0.1); // 10cm spacing
  DescartesTraj result;
  for (std::size_t i = 0; i < cart_path.size(); ++i)
  {
//    result.push_back( boost::make_shared<descartes_trajectory::AxialSymmetricPt>(cart_path[i], M_PI/6.0, descartes_trajectory::AxialSymmetricPt::Z_AXIS) );
    const Eigen::Affine3d& pose = cart_path[i];
    Eigen::Vector3d rpy = pose.rotation().eulerAngles(0,1,2);
    Eigen::Vector3d xyz = pose.translation();

//    descartes_core::TimingConstraint tm (0.3);

    double rx = rpy(0), ry = rpy(1), rz = rpy(2);
    double x = xyz(0), y = xyz(1), z = xyz(2);
    using namespace descartes_core;
    using namespace descartes_trajectory;
    TolerancedFrame frame (pose,
                           ToleranceBase::zeroTolerance<PositionTolerance>(x,y,z),
                           ToleranceBase::createSymmetric<OrientationTolerance>(rx,ry,rz,0,0,M_PI));
    result.push_back(TrajectoryPtPtr(new CartTrajectoryPt(frame, 0.0, M_PI/6.0)));
  }

  return result;
}


godel_process_planning::DescartesTraj
godel_process_planning::createJointPath(const std::vector<double> &start,
                                        const std::vector<double> &stop)
{
  JointVector path = interpolateJoint(start, stop, M_PI/180.0);
  DescartesTraj result;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    result.push_back(boost::make_shared<descartes_trajectory::JointTrajectoryPt>(path[i]));
  }
  return result;
}


trajectory_msgs::JointTrajectory godel_process_planning::getMoveitPlan(const std::string &group_name,
                                                                       const std::vector<double> &joints_start,
                                                                       const std::vector<double> &joints_stop,
                                                                       moveit::core::RobotModelConstPtr model)
{
  const moveit::core::JointModelGroup* group = model->getJointModelGroup(group_name);
  robot_state::RobotState goal_state(model);
  goal_state.setJointGroupPositions(group, joints_stop);

  moveit_msgs::GetMotionPlan::Request req;
  req.motion_plan_request.group_name = group_name;
  req.motion_plan_request.num_planning_attempts = 20;
  req.motion_plan_request.max_velocity_scaling_factor = 1.0;
  req.motion_plan_request.allowed_planning_time = 5.0; // seconds
  req.motion_plan_request.planner_id = "";//"RRTConnectkConfigDefault"; // empty -> default

  // Workspace params?
  req.motion_plan_request.workspace_parameters.header.frame_id = "world_frame";
  req.motion_plan_request.workspace_parameters.header.stamp = ros::Time::now();

  // Set the start state
  // Will want to add options here to start from a state that's not the start state
  moveit_msgs::RobotState start_state;
  start_state.is_diff = false;
  sensor_msgs::JointState joint_state;
  joint_state.name = group->getActiveJointModelNames();
  joint_state.position = joints_start;
  start_state.joint_state = joint_state;
  req.motion_plan_request.start_state = start_state;

  moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(goal_state, group);
  req.motion_plan_request.goal_constraints.push_back(c);

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

  trajectory_msgs::JointTrajectory jt;
  moveit_msgs::GetMotionPlan::Response res;
  if (client.call(req, res))
  {
    jt = res.motion_plan_response.trajectory.joint_trajectory;
  }
  return jt;
}

trajectory_msgs::JointTrajectory
godel_process_planning::planFreeMove(descartes_core::RobotModel& model,
                                     const std::string& group_name,
                                     moveit::core::RobotModelConstPtr moveit_model,
                                     const std::vector<double>& start,
                                     const std::vector<double>& stop)
{
  struct CollisionsGuard {
    CollisionsGuard(descartes_core::RobotModel& model) : model_(model) {
      model.setCheckCollisions(true);
      ROS_WARN_STREAM("Enabling collision");
    }
    ~CollisionsGuard() {
      model_.setCheckCollisions(false);
      ROS_WARN_STREAM("Disable collision");
    }
    descartes_core::RobotModel& model_;
  };

  CollisionsGuard guard (model);

  // Attempt joint interpolated motion
  DescartesTraj joint_approach = createJointPath(start, stop);
  // Check approach for collisions
  bool collision_free = true;
  for (std::size_t i = 0; i < joint_approach.size(); ++i)
  {
    if (!joint_approach[i]->isValid(model))
    {
      collision_free = false;
      break;
    }
  }

  if (collision_free)
  {
    return toROSTrajectory(joint_approach, model);
  }
  else
  {
    return godel_process_planning::getMoveitPlan(group_name, start, stop, moveit_model);
  }
}
