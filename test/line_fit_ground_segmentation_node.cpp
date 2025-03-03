#include <pcl/io/pcd_io.h>

#include "line_fit_ground_segmentation.h"

int main( )
{
  std::string cloud_file = "../assets/cloud/slope.pcd";
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_file, cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    return (-1);
  }
  pcl::io::loadPCDFile(cloud_file, cloud);
  GroundSegmentationParams params;
  GroundSegmentation segmenter(params);
  std::vector<int> labels;

  segmenter.estimateGround(cloud, &labels);
}