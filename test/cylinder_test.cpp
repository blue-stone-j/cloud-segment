
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "cylinder.h"

int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../assets/cloud/cylinder.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    return (-1);
  }

  //--------------------------------直通滤波-----------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z"); // 将Z轴不在（0，1.5）范围内的点过滤掉
  pass.setFilterLimits(0, 1.5);
  pass.filter(*cloud_filtered); // 剩余的点储存在cloud_filtered中后续使用
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size( ) << " data points." << std::endl;
  //--------------------------------计算法线-----------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setSearchMethod(tree);
  normal_est.setInputCloud(cloud_filtered);
  normal_est.setKSearch(50);
  normal_est.compute(*cloud_normals);
  //------------------------------创建分割对象---------------------------------
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
  // ------------------------点云分割，提取平面上的点--------------------------
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers_plane, *coefficients_plane); // 获取平面模型系数和平面上的点
  std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
  //----------------------------------提取平面---------------------------------
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>( ));
  extract.filter(*cloud_plane);
  std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size( ) << " data points." << std::endl;
  //-------------------------------提取圆柱体模型------------------------------
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
  // 获取平面以外的点和点的法线
  extract.setNegative(true);
  extract.filter(*cloud_filtered2);
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*normals2);
  // 为圆柱体分割创建分割对象，并设置参数
  seg.setOptimizeCoefficients(true);        // 设置对估计的模型系数需要进行优化
  seg.setModelType(pcl::SACMODEL_CYLINDER); // 设置分割模型为圆柱型
  seg.setMethodType(pcl::SAC_RANSAC);       // 设置采用RANSAC作为算法的参数估计方法
  seg.setNormalDistanceWeight(0.1);         // 设置表面法线权重系数
  seg.setMaxIterations(5000);               // 设置迭代的最大次数
  seg.setDistanceThreshold(0.05);           // 设置内点到模型的距离允许最大值
  seg.setRadiusLimits(0, 0.1);              // 设置估计出圆柱模型的半径范围
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(normals2);
  // 获取圆柱模型系数和圆柱上的点
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  //-----------------------------存储点云到输出文件----------------------------
  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty( ))
  {
    std::cout << "Can't find the cylindrical component." << std::endl;
  }
  else
  {
    std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size( ) << " data points." << std::endl;
  }
  return 0;
}