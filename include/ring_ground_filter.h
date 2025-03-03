#ifndef RING_GROUND_FILTER_H
#define RING_GROUND_FILTER_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <eigen3/Eigen/Dense>

#include "point_type.h"


// every ring can be labeled by three labels
enum Label
{
  GROUND,
  VERTICAL,
  UNKNOWN // Initial state, not classified
};

class GroundFilter
{
 public:
  GroundFilter( );
  void estimateGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &in_cloud_msg,
                      pcl::PointCloud<PointXYZIR> &ground_points);

 private:
  std::string point_topic_;
  std::string no_ground_topic, ground_topic;
  int sensor_model_;          // ring of lidar
  double sensor_height_;      // height from the ground
  double max_slope_;          // larger slope will reduce accuracy
  double vertical_threshold_; // threshold to judge whether is vertical
  bool floor_removal_;        // whether remove floor/ground

  int vertical_res_;          // num of ring
  int horizontal_res_;        // num of column
  Eigen::MatrixXi index_map_; // every element is point's index in this cloud
  Label class_label_[64];     // save label of every ring: ground, vertical and unknown
  // radius(xy) diff threshold of between neighbor rings: smaller->vertical; larger->ground
  double radius_table_[64];

  const int DEFAULT_HOR_RES = 2000; // default horizontal resolution

  void InitLabelArray(int in_model);
  void InitRadiusTable(int in_model);
  void InitDepthMap(int in_width);
  void FilterGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &in_cloud_msg,
                    pcl::PointCloud<PointXYZIR> &out_groundless_points,
                    pcl::PointCloud<PointXYZIR> &out_ground_points);
};


#endif