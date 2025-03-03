
// #include <iostream>
#include <algorithm>
#include "ray_ground_filter.h"



bool RayGroundFilter::TransformPointCloud(const std::string &in_target_frame,
                                          const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                                          const pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id)
  {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  Eigen::Affine3f mat;
  pcl::transformPointCloud(*in_cloud_ptr, *out_cloud_ptr, mat);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundFilter::ConvertXYZIToRTZColor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    const std::shared_ptr<PointCloudXYZIRT> &out_organized_points,
    const std::shared_ptr<std::vector<pcl::PointIndices>> &out_radial_divided_indices,
    const std::shared_ptr<std::vector<PointCloudXYZIRT>> &out_radial_ordered_clouds)
{
  out_organized_points->resize(in_cloud->points.size( ));
  out_radial_divided_indices->clear( );
  out_radial_divided_indices->resize(radial_dividers_num_);
  out_radial_ordered_clouds->resize(radial_dividers_num_);

  for (size_t i = 0; i < in_cloud->points.size( ); i++)
  {
    PointXYZIRT new_point;
    auto radius = static_cast<float>(
        sqrt(in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y));
    auto theta = static_cast<float>(atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI);
    if (theta < 0)
    {
      theta += 360;
    }
    if (theta >= 360)
    {
      theta -= 360;
    }

    auto radial_div = (size_t)floor(theta / radial_divider_angle_);

    auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

    new_point.point          = in_cloud->points[i];
    new_point.radius         = radius;
    new_point.theta          = theta;
    new_point.radial_div     = radial_div;
    new_point.concentric_div = concentric_div;
    new_point.original_index = i;

    out_organized_points->at(i) = new_point;

    // radial divisions
    out_radial_divided_indices->at(radial_div).indices.push_back(i);

    out_radial_ordered_clouds->at(radial_div).push_back(new_point);
  } // end for: divide all points into

// order radial points on each division
#pragma omp for
  for (size_t i = 0; i < radial_dividers_num_; i++)
  {
    std::sort(out_radial_ordered_clouds->at(i).begin( ), out_radial_ordered_clouds->at(i).end( ),
              [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; }); // NOLINT
  }
}

/*!
 * Classifies Points in the PointCloud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::ClassifyPointCloud(const std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                                         const pcl::PointIndices::Ptr &out_ground_indices,
                                         const pcl::PointIndices::Ptr &out_no_ground_indices)
{
  out_ground_indices->indices.clear( );
  out_no_ground_indices->indices.clear( );
#pragma omp for
  for (size_t i = 0; i < in_radial_ordered_clouds.size( ); i++) // sweep through each radial division
  {
    float prev_radius   = 0.f;
    float prev_height   = 0.f;
    bool prev_ground    = false;
    bool current_ground = false;
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size( ); j++) // loop through each point in the radial div
    {
      float points_distance          = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold         = tan(DEG2RAD(local_max_slope_)) * points_distance;
      float current_height           = in_radial_ordered_clouds[i][j].point.z;
      float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

      // for points which are very close causing the height threshold to be tiny, set a minimum value
      if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
      {
        height_threshold = min_height_threshold_;
      }

      // check current point height against the LOCAL threshold (previous point)
      if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
      {
        // Check again using general geometry (radius from origin) if previous points wasn't ground
        if (!prev_ground)
        {
          if (current_height <= general_height_threshold && current_height >= -general_height_threshold)
          {
            current_ground = true;
          }
          else
          {
            current_ground = false;
          }
        }
        else
        {
          current_ground = true;
        }
      }
      else // height difference is large
      {
        // check if current point is too far from previous one, if so classify again
        if (points_distance > reclass_distance_threshold_ && (current_height <= height_threshold && current_height >= -height_threshold))
        {
          current_ground = true;
        }
        else
        {
          current_ground = false;
        }
      }

      if (current_ground)
      {
        out_ground_indices->indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = true;
      }
      else
      {
        out_no_ground_indices->indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].point.z;
    } // endfor: have processed all points in this radial division
  }   // endfor: have processed all radial divisions
}

/*!
 * Removes the points higher than a threshold
 * @param in_cloud_ptr PointCloud to perform Clipping
 * @param in_clip_height Maximum allowed height in the cloud
 * @param out_clipped_cloud_ptr Resulting PointCloud with the points removed
 */
void RayGroundFilter::ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, const double in_clip_height,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud(in_cloud_ptr);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

#pragma omp for
  for (size_t i = 0; i < in_cloud_ptr->points.size( ); i++)
  {
    if (in_cloud_ptr->points[i].z > in_clip_height)
    {
      indices->indices.push_back(i);
    }
  }
  extractor.setIndices(indices);
  extractor.setNegative(true); // true removes the indices, false leaves only the indices
  extractor.filter(*out_clipped_cloud_ptr);
}

/*!
 * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
 * in the indices
 * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
 * @param in_indices Indices of the points to be both removed and kept
 * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
 * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
 */
void RayGroundFilter::ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                           const pcl::PointIndices &in_indices,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
  extract_ground.setInputCloud(in_cloud_ptr);
  pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(in_indices));
  extract_ground.setIndices(indices_ptr);

  extract_ground.setNegative(false); // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true); // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

/*!
 * Removes points up to a certain distance in the XY Plane
 * @param in_cloud_ptr Input PointCloud
 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
 * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
 */
void RayGroundFilter::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_min_distance,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud(in_cloud_ptr);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

#pragma omp for
  for (size_t i = 0; i < in_cloud_ptr->points.size( ); i++)
  {
    if (sqrt(in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x + in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y) < in_min_distance)
    {
      indices->indices.push_back(i);
    }
  }
  extractor.setIndices(indices);
  extractor.setNegative(true); // true removes the indices, false leaves only the indices
  extractor.filter(*out_filtered_cloud_ptr);
}

void RayGroundFilter::estimateGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  TransformPointCloud(base_frame_, in_sensor_cloud, current_sensor_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // remove points above certain point
  ClipCloud(current_sensor_cloud_ptr, clipping_height_, clipped_cloud_ptr);

  // remove closer points than a threshold
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  RemovePointsUpTo(clipped_cloud_ptr, min_point_distance_, filtered_cloud_ptr);

  std::shared_ptr<PointCloudXYZIRT> organized_points(new PointCloudXYZIRT);
  std::shared_ptr<std::vector<pcl::PointIndices>> radial_division_indices(new std::vector<pcl::PointIndices>);
  std::shared_ptr<std::vector<PointCloudXYZIRT>> radial_ordered_clouds(new std::vector<PointCloudXYZIRT>);

  radial_dividers_num_ = ceil(360 / radial_divider_angle_);

  ConvertXYZIToRTZColor(filtered_cloud_ptr, organized_points, radial_division_indices, radial_ordered_clouds);

  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices), no_ground_indices(new pcl::PointIndices);

  ClassifyPointCloud(*radial_ordered_clouds, ground_indices, no_ground_indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  ExtractPointsIndices(filtered_cloud_ptr, *ground_indices, ground_cloud_ptr, no_ground_cloud_ptr);
}

RayGroundFilter::RayGroundFilter( )
{
  Run( );
}

void RayGroundFilter::Run( )
{
  // Model   |   Horizontal   |   Vertical   | FOV(Vertical)    degrees / rads
  // ----------------------------------------------------------
  // HDL-64  |0.08-0.35(0.32) |     0.4      |  -24.9 <=x<=2.0   (26.9  / 0.47)
  // HDL-32  |     0.1-0.4    |     1.33     |  -30.67<=x<=10.67 (41.33 / 0.72)
  // VLP-16  |     0.1-0.4    |     2.0      |  -15.0<=x<=15.0   (30    / 0.52)
  // VLP-16HD|     0.1-0.4    |     1.33     |  -10.0<=x<=10.0   (20    / 0.35)

  base_frame_                  = "base_link";
  general_max_slope_           = 3.0;
  local_max_slope_             = 5.0;
  radial_divider_angle_        = 0.1;  // 1 degree default
  concentric_divider_distance_ = 0.0;  // 0.0 meters default
  min_height_threshold_        = 0.05; // 0.05 meters default
  clipping_height_             = 2.0;  // 2.0 meters default above the car
  min_point_distance_          = 1.85; // 1.85 meters default
  reclass_distance_threshold_  = 0.2;  // 0.5 meters default

  // returns the smallest possible integer value which is greater than or equal to the given argument
  radial_dividers_num_ = ceil(360 / radial_divider_angle_);

  std::string no_ground_topic, ground_topic;
  no_ground_topic = "/points_no_ground";

  ground_topic = "/points_ground";
}
