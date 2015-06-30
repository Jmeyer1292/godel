#include "godel_simple_gui/states/wait_to_simulate_state.h"
// previous state
#include "godel_simple_gui/states/surface_select_state.h"
// next state
#include "godel_simple_gui/states/simulating_state.h"
// reset state
#include "godel_simple_gui/states/scan_teach_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

#include "godel_msgs/GetAvailableMotionPlans.h"

#include <QtConcurrentRun>

const static std::string GET_AVAILABLE_MOTION_PLANS_SERVICE = "get_available_motion_plans";

void godel_simple_gui::WaitToSimulateState::onStart(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToSimulateState start");
  gui.setText("Ready to Simulate.\nPress 'Next' to observe the plan.");

  ros::ServiceClient client =
      gui.nodeHandle().serviceClient<godel_msgs::GetAvailableMotionPlans>(GET_AVAILABLE_MOTION_PLANS_SERVICE);

  godel_msgs::GetAvailableMotionPlans srv;
  if (client.call(srv))
  {
    this->plan_names_ = srv.response.names;
  }
  else
  {
    ROS_WARN_STREAM("Could not fetch plan names");
  }
}

void godel_simple_gui::WaitToSimulateState::onExit(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToSimulateState exit");
}

// Handlers for the fixed buttons
void godel_simple_gui::WaitToSimulateState::onNext(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToSimulateState next");
  Q_EMIT newStateAvailable( new SimulatingState(plan_names_) );
}

void godel_simple_gui::WaitToSimulateState::onBack(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToSimulateState back");
  Q_EMIT newStateAvailable( new SurfaceSelectState() );
}

void godel_simple_gui::WaitToSimulateState::onReset(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToSimulateState reset");
  Q_EMIT newStateAvailable( new ScanTeachState() );
}



