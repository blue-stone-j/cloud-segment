#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

class EuclideanCluster
{
 public:
  EuclideanCluster( );
  ~EuclideanCluster( );
  EuclideanCluster(int minNum, int maxNum, double clusterTolerance); // size limit of cluster, distance tolerance
  // get index of points and size of this cloud
  void computeEuclideanCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<pcl::PointIndices> &cluster_indices);

 public:
  int minNum             = 30;
  int maxNum             = 20000;
  float clusterTolerance = 0.4;

 private:
  bool initCompute(const pcl::PointCloud<pcl::PointXYZ> &cloud_in);
  pcl::PointIndices indices;
  size_t size_pc;
};

#endif