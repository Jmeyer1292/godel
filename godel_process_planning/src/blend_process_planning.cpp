#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

namespace godel_process_planning
{


bool ProcessPlanningManager::handleBlendPlanning(godel_msgs::BlendProcessPlanning::Request& req,
                                                 godel_msgs::BlendProcessPlanning::Response& res)
{
  // Use request parameters to configure a robot model
  //

  ROS_WARN_STREAM("Not implemented");
  return false;
}

}
