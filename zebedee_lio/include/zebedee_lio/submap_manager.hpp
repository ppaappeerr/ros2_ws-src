#ifndef ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_
#define ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <memory>

// PCL 포인트 타입 정의
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

namespace zebedee_lio
{

class SubmapManager
{
public:
  SubmapManager(double sliding_window_size, double voxel_leaf_size);
  ~SubmapManager();

  void updateSlidingWindow(const Eigen::Vector3d& current_position);
  void addPointCloud(const PointCloud::Ptr& cloud_to_add);
  PointCloud::Ptr getSubmap();

private:
  double sliding_window_size_;
  double voxel_leaf_size_; 
  
  PointCloud::Ptr submap_cloud_;
  pcl::VoxelGrid<PointType> voxel_grid_filter_;
  pcl::CropBox<PointType> crop_box_filter_;
};

}

#endif
