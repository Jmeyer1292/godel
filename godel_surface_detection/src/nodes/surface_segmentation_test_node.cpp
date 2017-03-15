#include <segmentation/surface_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
extractSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  static const float INPUT_CLOUD_VOXEL_FILTER_SIZE = 0.0015;

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(INPUT_CLOUD_VOXEL_FILTER_SIZE,
                  INPUT_CLOUD_VOXEL_FILTER_SIZE,
                  INPUT_CLOUD_VOXEL_FILTER_SIZE);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  vox.filter(*process_cloud);

  SurfaceSegmentation ss(process_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
  ss.computeSegments(output);

  return output;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"surface_segmentation_test_node");
  ros::NodeHandle pnh ("~");

  std::string filename;
  if (!pnh.getParam("filename", filename))
  {
    ROS_ERROR("Node requires user to set private parameter 'filename'");
    return 1;
  }

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  if (pcl::io::loadPCDFile(filename, *cloud) < 0)
  {
    ROS_ERROR("Could not load cloud file: %s", filename.c_str());
    return 2;
  }

  ROS_INFO("Starting surface extraction routine...");
  auto start_tm = ros::Time::now();
  auto surface_cloud = extractSurfaces(cloud);
  auto finish_tm = ros::Time::now();
  ROS_INFO("Surface extract completed after %f seconds.", (finish_tm - start_tm).toSec());

  pcl::io::savePCDFile("surfaces.pcd", *surface_cloud);
  return 0;
}
