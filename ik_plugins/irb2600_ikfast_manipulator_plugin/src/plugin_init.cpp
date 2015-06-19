//register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <irb2600_ikfast_manipulator_plugin/abb_irb2600_manipulator_ikfast_moveit_plugin.hpp> 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(irb2600_ikfast_manipulator_plugin::IKFastKinematicsPlugin, kinematics::KinematicsBase);
