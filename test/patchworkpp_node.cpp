
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "point_type.h"

#include "patchworkpp.h"


int main(int argc, char **argv)
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file your_point_cloud_file.pcd \n");
    return (-1);
  }

  Eigen::MatrixXf cloud_in;
  cloud_in.resize(cloud->size( ), 4);
  for (size_t i = 0; i < cloud->size( ); i++)
  {
    cloud_in(i, 0) = cloud->points[i].x;
    cloud_in(i, 1) = cloud->points[i].y;
    cloud_in(i, 2) = cloud->points[i].z;
    cloud_in(i, 3) = cloud->points[i].intensity;
  }

  patchwork::Params params;
  patchwork::PatchWorkpp pwpp(params);

  pwpp.estimateGround(cloud_in);

  return 0;
}
