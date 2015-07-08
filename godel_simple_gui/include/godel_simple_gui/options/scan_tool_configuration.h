#ifndef SCAN_TOOL_CONFIGURATION_H
#define SCAN_TOOL_CONFIGURATION_H

#include "godel_simple_gui/parameter_window_base.h"

#include "godel_msgs/ScanPlanParameters.h"

namespace Ui {
  class ScanToolConfigWindow;
}

namespace godel_simple_gui
{

class ScanPlanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  ScanPlanConfigWidget(godel_msgs::ScanPlanParameters params);
  godel_msgs::ScanPlanParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();
  
  static QStringList quality_metric_list;

protected:
  virtual int get_quality_combobox_index();

  godel_msgs::ScanPlanParameters params_;
  Ui::ScanToolConfigWindow* ui_;
};

}

#endif // SCAN_TOOL_CONFIGURATION_H
