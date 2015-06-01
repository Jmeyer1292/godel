#ifndef MACRO_SCAN_H
#define MACRO_SCAN_H

#include <geometry_msgs/Pose.h>
#include <path_editor/path_editor.h>
#include <vector>

namespace godel_surface_detection
{
namespace scan
{

class MacroScanManager
{
public:
  MacroScanManager();

  /**
   * @brief Turn on interactive markers that allow one to train
   *        a robot path
   */
  void enablePathEditing();
  /**
   * @brief Disable interactive markers (such that they are hidden)
   *        Last information still available elsewhere
   */
  void disablePathEditing();

  const std::vector<geometry_msgs::Pose>& getUserPoses() const;

private:
  void onUserUpdate(const std::vector<geometry_msgs::Pose>& poses);

  path_editor::PoseManipulator pose_edit_; // controls points
  ros::Publisher marker_pub_; // displays the resulting path
};


} // namespace scan
} // namespace godel_surface_detection

#endif // MACRO_SCAN_H

