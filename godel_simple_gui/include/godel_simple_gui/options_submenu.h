#ifndef OPTIONS_SUBMENU_H
#define OPTIONS_SUBMENU_H

#include <QWidget>

#include "godel_simple_gui/options/robot_scan_configuration.h"
#include "godel_simple_gui/options/surface_detection_configuration.h"

namespace Ui {
  class OptionsSubmenu;
}

namespace godel_simple_gui
{

class OptionsSubmenu : public QWidget
{
  Q_OBJECT

public:
  OptionsSubmenu(QWidget* parent = 0);

  // For every submenu / set of parameters, we have getters/setters
  const godel_msgs::RobotScanParameters& robotScanParams() const;
  void setRobotScanParams(const godel_msgs::RobotScanParameters& params);

  const godel_msgs::SurfaceDetectionParameters& surfaceDetectionParams() const;
  void setSurfaceDetectionParams(const godel_msgs::SurfaceDetectionParameters& params);

Q_SIGNALS:
  void saveRequested();

private:
  // Display layout
  Ui::OptionsSubmenu* ui_;
  // Configuration components
  RobotScanConfigWidget* robot_scan_;
  SurfaceDetectionConfigWidget* surface_detection_;

};

} // end namespace godel_simple_gui

#endif // OPTIONS_SUBMENU_H

