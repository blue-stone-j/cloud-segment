/*
 * ring_ground_filter.cpp
 *
 * Created on  : June 5, 2018
 * Author  : Patiphon Narksri
 *
 */


#include "ring_ground_filter.h"

int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
  if (pcl::io::loadPCDFile<PointXYZIR>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file your_point_cloud_file.pcd \n");
    return (-1);
  }

  GroundFilter node;

  node.Callback(cloud);

  return 0;
}
