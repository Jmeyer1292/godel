#include "godel_process_planning/godel_process_planning.h"


godel_process_planning::ProcessPlanningManager::ProcessPlanningManager(const std::string &world_frame,
                                                                       const std::string &blend_group,
                                                                       const std::string &blend_tcp,
                                                                       const std::string &keyence_group,
                                                                       const std::string &keyence_tcp,
                                                                       const std::string &ik_plugin)
  : model_loader_("descartes_core", "descartes_core::RobotModel")
{
  blend_model_ = model_loader_.createInstance(ik_plugin);
  blend_model_->initialize("robot_description", blend_group, world_frame, blend_tcp);

  keyence_model_ = model_loader_.createInstance(ik_plugin);
  keyence_model_->initialize("robot_description", keyence_group, world_frame, keyence_tcp);
}
