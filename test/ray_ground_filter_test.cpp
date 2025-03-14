/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 ********************
 * amc-nu (abrahammonrroy@yahoo.com)
 */

#include <pcl/io/pcd_io.h>

#include "ray_ground_filter.h"

int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    return (-1);
  }

  RayGroundFilter app;

  app.estimateGround(cloud);

  return 0;
}
