#include <pcl/io/pcd_io.h>

#include "euclidean_cluster.h"

int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../assets/cloud/fog.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    return (-1);
  }

  // pcl
  // 创建用于查找的Kd树对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // 初始化聚类提取对象
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.4); // 设置临近点的搜索半径
  ec.setMinClusterSize(30);    // 设置一个聚类需要的最小点数
  ec.setMaxClusterSize(20000); // 设置一个聚类需要的最大点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::cout << "---pcl---" << std::endl;
  std::sort(cluster_indices.begin( ), cluster_indices.end( ),
            [](pcl::PointIndices a, pcl::PointIndices b) { return a.indices.size( ) < b.indices.size( ); });
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin( ); it != cluster_indices.end( ); ++it)
  {
    std::cout << it->indices.size( ) << std::endl;
  }


  // customized
  EuclideanCluster euc(30, 20000, 0.4);
  cluster_indices.clear( );
  euc.computeEuclideanCluster(*cloud, cluster_indices);

  std::cout << "---cum---" << std::endl;
  std::sort(cluster_indices.begin( ), cluster_indices.end( ),
            [](pcl::PointIndices a, pcl::PointIndices b) { return a.indices.size( ) < b.indices.size( ); });
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin( ); it != cluster_indices.end( ); ++it)
  {
    std::cout << it->indices.size( ) << std::endl;
  }

  return 0;
}