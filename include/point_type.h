#ifndef POINT_TYPE_H
#define POINT_TYPE_H

#include <pcl/point_types.h>


struct PointXYZIR
{
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(std::uint16_t, ring, ring))

#endif