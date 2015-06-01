#include <godel_surface_detection/scan/macro_scan.h>

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>


static visualization_msgs::MarkerArray
makeArrayMarkerArray(const std::vector<geometry_msgs::Pose>& poses)
{
  visualization_msgs::MarkerArray array;
  for (size_t i = 0; i < poses.size(); ++i)
  {
//    Eigen::AngleAxisd angleaxis (M_PI/2.0, Eigen::Vector3d(0,1,0));
//    Eigen::Quaterniond new_q = poses[i] * angleaxis;

//    geometry_msgs::Pose pose;
//    tf::pointEigenToMsg(poses[i], pose.position); // copy point
//    tf::quaternionEigenToMsg(new_q, pose.orientation);

    visualization_msgs::Marker arrow;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.scale.x = 0.1;
    arrow.scale.y = 0.02;
    arrow.scale.z = 0.02;
    arrow.color.r = 1.0;
    arrow.color.a = 1;
    arrow.header.frame_id = "world_frame";
    arrow.header.stamp = ros::Time::now();
    arrow.pose = poses[i];
    arrow.ns = "arrow";
    arrow.id = i;
    arrow.action = visualization_msgs::Marker::ADD;

    array.markers.push_back(arrow);
  }
  return array;
}


godel_surface_detection::scan::MacroScanManager::MacroScanManager()
  : pose_edit_("pose_editor")
{
  ros::NodeHandle nh ("pose_editor");
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("camera_poses", 1);
}

void godel_surface_detection::scan::MacroScanManager::enablePathEditing()
{
  // initialize three control points
  std::vector<geometry_msgs::Pose> poses;

  geometry_msgs::Pose p;
  p.position.z = 1.0;
  p.position.x = 1.0;
  p.position.y = -1.0;
  poses.push_back(p);
  p.position.y = 0.0;
  poses.push_back(p);
  p.position.y = -1.0;
  poses.push_back(p);

  boost::function<void(std::vector<geometry_msgs::Pose> const&)> cb = boost::bind(&MacroScanManager::onUserUpdate, this, _1);
  pose_edit_.setPoses(poses);
  pose_edit_.setCallback(cb);
}

void godel_surface_detection::scan::MacroScanManager::disablePathEditing()
{
  // clear server
  pose_edit_.setPoses(std::vector<geometry_msgs::Pose>());
}

const std::vector<geometry_msgs::Pose>& godel_surface_detection::scan::MacroScanManager::getUserPoses() const
{
  return pose_edit_.currentPoses();
}


void godel_surface_detection::scan::MacroScanManager::onUserUpdate(const std::vector<geometry_msgs::Pose>& poses)
{
// Calculate curved path
  // To Eigen for math
  Eigen::Affine3d p1, p2, p3;
  tf::poseMsgToEigen(poses[0], p1);
  tf::poseMsgToEigen(poses[1], p2);
  tf::poseMsgToEigen(poses[2], p3);

  // Calculate curve through space
  path_editor::VecAffine3d curve = path_editor::bezierInterpolate(p1, p2, p3, 5);

  // Back to geometry msgs
  std::vector<geometry_msgs::Pose> msgs;
  for (std::size_t i = 0; i < curve.size(); ++i)
  {
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(curve[i], p);
    msgs.push_back(p);
  }

  // Now build marker array message
  visualization_msgs::MarkerArray array = makeArrayMarkerArray(msgs);
  // and publish
  marker_pub_.publish(array);
}
