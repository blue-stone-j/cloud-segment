#include "linefit_ground_segmentation.h"

#include <pcl/io/pcd_io.h>

int main( )
{
  std::string cloud_file;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile(cloud_file, cloud);
  GroundSegmentationParams params;
  GroundSegmentation segmenter(params);
  std::vector<int> labels;

  segmenter.segment(cloud, &labels);
}