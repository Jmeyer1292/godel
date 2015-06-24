#include "godel_process_planning/godel_process_planning.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

godel_process_planning::ProcessPlanningManager::ProcessPlanningManager(const std::string &world_frame,
                                                                       const std::string &blend_group,
                                                                       const std::string &blend_tcp,
                                                                       const std::string &keyence_group,
                                                                       const std::string &keyence_tcp,
                                                                       const std::string &ik_plugin)
  : plugin_loader_("descartes_core", "descartes_core::RobotModel")
{
  blend_model_ = plugin_loader_.createInstance(ik_plugin);
  blend_model_->initialize("robot_description", blend_group, world_frame, blend_tcp);

  keyence_model_ = plugin_loader_.createInstance(ik_plugin);
  keyence_model_->initialize("robot_description", keyence_group, world_frame, keyence_tcp);

  // Load the moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit_model_ = robot_model_loader.getModel();

  if (moveit_model_.get() == NULL) throw std::runtime_error("Could not load moveit robot model");
}
