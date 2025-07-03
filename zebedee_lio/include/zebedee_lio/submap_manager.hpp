#ifndef ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_
#define ZEBEDEE_LIO__SUBMAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ikd-Tree/ikd_Tree.h" // ikd-Tree 헤더

#include <Eigen/Core>
#include <memory>

// PCL 포인트 타입 및 ikd-Tree 벡터 타입 정의
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointVector = KD_TREE<PointType>::PointVector; // ikd-Tree가 사용하는 특수 벡터

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
  KD_TREE<PointType>::Ptr getTree();

private:
  KD_TREE<PointType>::Ptr ikd_tree_;
  double sliding_window_size_;
  
  // 🚨 [에러 수정] 누락된 멤버 변수 선언 추가
  double voxel_leaf_size_; 
};

}

#endif