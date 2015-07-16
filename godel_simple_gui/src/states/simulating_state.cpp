#include "godel_simple_gui/states/simulating_state.h"
// prev
#include "godel_simple_gui/states/wait_to_simulate_state.h"
 //next
#include "godel_simple_gui/states/wait_to_execute_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

#include "godel_msgs/SelectMotionPlan.h"

#include <QtConcurrentRun>

const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";

godel_simple_gui::SimulatingState::SimulatingState(const std::vector<std::string> &plans)
  : plan_names_(plans)
{}

void godel_simple_gui::SimulatingState::onStart(BlendingWidget& gui)
{
  ROS_INFO_STREAM("SimulatingState start");
  gui.setText("Simulating...");
  gui.setButtonsEnabled(false);

  sim_client_ = gui.nodeHandle().serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
  QtConcurrent::run(this, &SimulatingState::simulateAll);
}

void godel_simple_gui::SimulatingState::onExit(BlendingWidget& gui)
{
  ROS_INFO_STREAM("SimulatingState exit");
  gui.setButtonsEnabled(true);
}

// Handlers for the fixed buttons
void godel_simple_gui::SimulatingState::onNext(BlendingWidget& gui)
{
  ROS_INFO_STREAM("SimulatingState next");
}

void godel_simple_gui::SimulatingState::onBack(BlendingWidget& gui)
{
  ROS_INFO_STREAM("SimulatingState back");
}

void godel_simple_gui::SimulatingState::onReset(BlendingWidget& gui)
{
  ROS_INFO_STREAM("SimulatingState reset");
}

void godel_simple_gui::SimulatingState::simulateAll()
{
  for (std::size_t i = 0; i < plan_names_.size(); ++i)
  {
    simulateOne(plan_names_[i]);
  }

  Q_EMIT newStateAvailable( new WaitToExecuteState(plan_names_) );
}

void godel_simple_gui::SimulatingState::simulateOne(const std::string &plan)
{
  ROS_INFO_STREAM("Simulating " << plan);
  godel_msgs::SelectMotionPlan srv;
  srv.request.name = plan;
  srv.request.simulate = true;
  srv.request.wait_for_execution = true;
  if (!sim_client_.call(srv))
  {
    ROS_WARN_STREAM("Client simulation request returned false");
  }
}





