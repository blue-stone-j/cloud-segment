/*
 * ring_ground_filter.cpp
 *
 * Created on  : June 5, 2018
 * Author  : Patiphon Narksri
 *
 */

#include <pcl/io/pcd_io.h>

#include "ring_ground_filter.h"

int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
  if (pcl::io::loadPCDFile<PointXYZIR>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    return (-1);
  }

  GroundFilter node;
  pcl::PointCloud<PointXYZIR> ground_points;
  node.estimateGround(cloud, ground_points);

  return 0;
}
