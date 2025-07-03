#include "zebedee_lio/submap_manager.hpp"

namespace zebedee_lio
{

SubmapManager::SubmapManager(double sliding_window_size, double voxel_leaf_size)
: sliding_window_size_(sliding_window_size)
{
  // ikd-Tree 초기화
  ikd_tree_ = std::make_shared<KD_TREE<PointType>>(voxel_leaf_size);
}

SubmapManager::~SubmapManager() {}

void SubmapManager::updateSlidingWindow(const Eigen::Vector3d& current_position)
{
  // 현재 위치를 중심으로 슬라이딩 윈도우 박스(Bounding Box)를 정의
  sliding_window_box_.vertex_min[0] = current_position.x() - sliding_window_size_;
  sliding_window_box_.vertex_min[1] = current_position.y() - sliding_window_size_;
  sliding_window_box_.vertex_min[2] = current_position.z() - sliding_window_size_;
  sliding_window_box_.vertex_max[0] = current_position.x() + sliding_window_size_;
  sliding_window_box_.vertex_max[1] = current_position.y() + sliding_window_size_;
  sliding_window_box_.vertex_max[2] = current_position.z() + sliding_window_size_;

  // ikd-Tree의 Box-wise 삭제 기능을 사용하여 윈도우 밖의 포인트들을 효율적으로 제거
  points_to_delete_.clear();
  
  // Box_Search 함수 호출 (올바른 타입 사용)
  ikd_tree_->Box_Search(sliding_window_box_, points_to_delete_);

  // 슬라이딩 윈도우 밖의 포인트들 삭제
  if (!points_to_delete_.empty())
  {
    // Delete_Point_Boxes 함수 사용 (public 함수)
    vector<BoxPointType> boxes_to_delete;
    boxes_to_delete.push_back(sliding_window_box_);
    ikd_tree_->Delete_Point_Boxes(boxes_to_delete);
  }
}

void SubmapManager::addPointCloud(const PointCloud::Ptr& cloud_to_add)
{
  if (cloud_to_add->points.empty())
  {
    return;
  }
  // ikd-Tree에 포인트 클라우드 추가
  ikd_tree_->Build(cloud_to_add->points);
}

PointCloud::Ptr SubmapManager::getSubmap()
{
  PointCloud::Ptr submap_cloud(new PointCloud());
  
  // ikd-Tree에서 현재 유효한 모든 포인트를 가져와서 반환
  // flatten 함수 호출 (올바른 파라미터 3개)
  ikd_tree_->flatten(ikd_tree_->Root_Node, submap_cloud->points, delete_point_storage_set::NOT_RECORD);
  
  return submap_cloud;
}

KD_TREE<PointType>::Ptr SubmapManager::getTree()
{
  return ikd_tree_;
}

} // namespace zebedee_lio