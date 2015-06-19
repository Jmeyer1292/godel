#ifndef GODEL_PROCESS_PLANNING_H
#define GODEL_PROCESS_PLANNING_H

#include "godel_msgs/BlendProcessPlanning.h"
#include "godel_msgs/KeyenceProcessPlanning.h"

#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

namespace godel_process_planning
{

class ProcessPlanningManager
{
public:
  ProcessPlanningManager(const std::string& world_frame, const std::string& blend_group,
                         const std::string& blend_tcp, const std::string& keyence_group,
                         const std::string& keyence_tcp, const std::string& ik_plugin);

  bool handleBlendPlanning(godel_msgs::BlendProcessPlanning::Request& req,
                           godel_msgs::BlendProcessPlanning::Response& res);

  bool handleKeyencePlanning(godel_msgs::KeyenceProcessPlanning::Request& req,
                             godel_msgs::KeyenceProcessPlanning::Response& res);
private:
  descartes_core::RobotModelPtr blend_model_;
  descartes_core::RobotModelPtr keyence_model_;
  pluginlib::ClassLoader<descartes_core::RobotModel> model_loader_;
};

}

#endif
