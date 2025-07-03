#include "zebedee_lio/submap_manager.hpp"

namespace zebedee_lio
{

SubmapManager::SubmapManager(double sliding_window_size, double voxel_leaf_size) 
: sliding_window_size_(sliding_window_size), voxel_leaf_size_(voxel_leaf_size)
{
  ikd_tree_ = std::make_shared<KD_TREE<PointType>>(voxel_leaf_size_);
}

SubmapManager::~SubmapManager() {}

void SubmapManager::updateSlidingWindow(const Eigen::Vector3d& current_position) 
{
  // 🚨 [에러 수정] ikd-Tree가 요구하는 PointVector 타입 사용
  PointVector all_points;
  ikd_tree_->flatten(ikd_tree_->Root_Node, all_points, NOT_RECORD);

  if (all_points.empty()) {
    return;
  }
  
  PointVector points_to_keep;
  points_to_keep.reserve(all_points.size());

  for (const auto& pt : all_points) {
    float dist_x = pt.x - current_position.x();
    float dist_y = pt.y - current_position.y();
    float dist_z = pt.z - current_position.z();
    if (std::abs(dist_x) < sliding_window_size_ &&
        std::abs(dist_y) < sliding_window_size_ &&
        std::abs(dist_z) < sliding_window_size_)
    {
      points_to_keep.push_back(pt);
    }
  }
  
  // 🚨 [에러 수정] Clear() 함수 대신 reset()으로 트리 재구성
  if (points_to_keep.size() < 1) { // 유지할 포인트가 없으면 비어있는 트리로 초기화
      ikd_tree_.reset(new KD_TREE<PointType>(voxel_leaf_size_));
  } else {
      ikd_tree_.reset(new KD_TREE<PointType>(voxel_leaf_size_));
      ikd_tree_->Build(points_to_keep);
  }
}

void SubmapManager::addPointCloud(const PointCloud::Ptr& cloud_to_add) 
{
  if (cloud_to_add->points.empty()) 
  {
    return;
  }
  // 🚨 [에러 수정] Build 함수에 맞게 PointCloud의 points 벡터를 직접 전달
  ikd_tree_->Build(cloud_to_add->points);
}

PointCloud::Ptr SubmapManager::getSubmap() 
{
  PointCloud::Ptr submap_cloud(new PointCloud());
  PointVector submap_points;
  ikd_tree_->flatten(ikd_tree_->Root_Node, submap_points, NOT_RECORD);
  
  // PointVector를 PointCloud로 변환
  submap_cloud->points = submap_points;
  return submap_cloud;
}

KD_TREE<PointType>::Ptr SubmapManager::getTree() 
{
  return ikd_tree_;
}

}