#include "godel_simple_gui/states/scan_teach_state.h"
#include "godel_simple_gui/states/scanning_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"


void godel_simple_gui::ScanTeachState::onStart(BlendingWidget& gui)
{
  ROS_INFO_STREAM("ScanTeachState start");
  gui.setText("Ready to Scan\nPress Next to Continue");
}

void godel_simple_gui::ScanTeachState::onExit(BlendingWidget& gui)
{
  ROS_INFO_STREAM("ScanTeachState exit");
}

// Handlers for the fixed buttons
void godel_simple_gui::ScanTeachState::onNext(BlendingWidget& gui)
{
  ROS_INFO_STREAM("ScanTeachState next");
  Q_EMIT newStateAvailable( new ScanningState() );
}

void godel_simple_gui::ScanTeachState::onBack(BlendingWidget& gui)
{
  ROS_INFO_STREAM("ScanTeachState back");
}

void godel_simple_gui::ScanTeachState::onReset(BlendingWidget& gui)
{
  ROS_INFO_STREAM("ScanTeachState reset");
}

