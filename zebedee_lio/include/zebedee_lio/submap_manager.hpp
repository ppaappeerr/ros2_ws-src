#ifndef ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_
#define ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <memory>

// ikd-Tree 헤더 (올바른 경로)
#include "ikd-Tree/ikd_Tree.h"

// PCL 포인트 타입 정의
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

namespace zebedee_lio
{

class SubmapManager
{
public:
  // 생성자: 슬라이딩 윈도우의 크기와 서브샘플링 해상도를 설정
  SubmapManager(double sliding_window_size, double voxel_leaf_size);
  ~SubmapManager();

  // 현재 로봇의 위치를 기준으로 슬라이딩 윈도우를 업데이트
  void updateSlidingWindow(const Eigen::Vector3d& current_position);

  // 새로운 포인트 클라우드를 서브맵에 추가
  void addPointCloud(const PointCloud::Ptr& cloud_to_add);

  // 현재 서브맵을 PointCloud 형태로 반환
  PointCloud::Ptr getSubmap();

  // 외부에서 ikd-Tree에 직접 접근하기 위한 포인터 반환
  KD_TREE<PointType>::Ptr getTree();

private:
  // ikd-Tree 객체
  KD_TREE<PointType>::Ptr ikd_tree_;

  // 슬라이딩 윈도우의 크기 (반경)
  double sliding_window_size_;

  // 슬라이딩 윈도우 영역을 정의하는 박스
  BoxPointType sliding_window_box_;
  
  // 삭제할 포인트를 담아두는 벡터 (올바른 타입 사용)
  std::vector<PointType, Eigen::aligned_allocator<PointType>> points_to_delete_;
};

} // namespace zebedee_lio

#endif // ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_