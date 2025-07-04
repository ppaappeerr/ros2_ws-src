#include "zebedee_lio/submap_manager.hpp"

namespace zebedee_lio
{

SubmapManager::SubmapManager(double sliding_window_size, double voxel_leaf_size) 
: sliding_window_size_(sliding_window_size), voxel_leaf_size_(voxel_leaf_size)
{
  // Initialize submap point cloud and filters
  submap_cloud_ = std::make_shared<PointCloud>();
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

SubmapManager::~SubmapManager() {}

void SubmapManager::addPointCloud(const PointCloud::Ptr& cloud_in)
{
  if (cloud_in->points.empty()) {
    return;
  }
  
  // Add new points to existing submap
  *submap_cloud_ += *cloud_in;
  
  // Downsample entire submap to maintain consistent point density
  PointCloud::Ptr filtered_cloud(new PointCloud());
  voxel_grid_filter_.setInputCloud(submap_cloud_);
  voxel_grid_filter_.filter(*filtered_cloud);
  submap_cloud_ = filtered_cloud;
}

void SubmapManager::updateSlidingWindow(const Eigen::Vector3d& current_position) 
{
  if (submap_cloud_->points.empty()) {
    return;
  }
  
  // Set CropBox filter region centered on current position
  Eigen::Vector4f min_pt, max_pt;
  min_pt << static_cast<float>(current_position.x() - sliding_window_size_),
            static_cast<float>(current_position.y() - sliding_window_size_),
            static_cast<float>(current_position.z() - sliding_window_size_),
            1.0f;
  max_pt << static_cast<float>(current_position.x() + sliding_window_size_),
            static_cast<float>(current_position.y() + sliding_window_size_),
            static_cast<float>(current_position.z() + sliding_window_size_),
            1.0f;
            
  crop_box_filter_.setMin(min_pt);
  crop_box_filter_.setMax(max_pt);
  
  // Use CropBox filter to efficiently remove points outside sliding window
  PointCloud::Ptr cropped_cloud(new PointCloud());
  crop_box_filter_.setInputCloud(submap_cloud_);
  crop_box_filter_.filter(*cropped_cloud);
  submap_cloud_ = cropped_cloud;
}

PointCloud::Ptr SubmapManager::getSubmap() 
{
  return submap_cloud_;
}

// ikd-Tree functionality replaced with PCL-based approach
// KD_TREE<PointType>::Ptr SubmapManager::getTree() 
// {
//   return nullptr; 
// }

}
