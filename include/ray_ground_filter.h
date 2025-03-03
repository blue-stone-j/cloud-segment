
#ifndef RAY_GROUND_FILTER_RAY_GROUND_FILTER_H
#define RAY_GROUND_FILTER_RAY_GROUND_FILTER_H


#include <vector>
#include <memory>
#include <string>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>



class RayGroundFilter
{
 private:
  std::string base_frame_;

  double general_max_slope_;           // degrees: judge height
  double local_max_slope_;             // degrees: judge height difference between two points
  double radial_divider_angle_;        // distance in rads between dividers
  double concentric_divider_distance_; // distance in meters between concentric divisions
  double min_height_threshold_;        // minimum height threshold regardless the slope, useful for close points
  double clipping_height_;             // the points higher than this will be removed from the input cloud.
  double min_point_distance_;          // minimum distance from the origin to consider a point as valid
  double reclass_distance_threshold_;  // distance between points at which re classification will occur

  size_t radial_dividers_num_;
  // size_t concentric_dividers_num_;

  struct PointXYZIRT
  {
    pcl::PointXYZI point;

    float radius; // cylindrical coords on XY Plane
    float theta;  // angle deg on XY plane

    size_t radial_div;     // (theta)index of the radial dvision to which this point belongs to
    size_t concentric_div; // (radius)index of the concentric division to which this points belongs to

    size_t original_index; // index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZIRT> PointCloudXYZIRT;

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform success
   * @retval false transform failed
   */
  bool TransformPointCloud(const std::string &in_target_frame, const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr);

  /*!
   * calculate index of radial & concentric, theta
   * @param[in] in_cloud Input Point Cloud to be organized in radial segments
   * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
   * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
   * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
   */
  void ConvertXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                             const std::shared_ptr<PointCloudXYZIRT> &out_organized_points,
                             const std::shared_ptr<std::vector<pcl::PointIndices>> &out_radial_divided_indices,
                             const std::shared_ptr<std::vector<PointCloudXYZIRT>> &out_radial_ordered_clouds);

  /*!
   * Classifies Points in the PointCloud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
   * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
   * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
   */
  void ClassifyPointCloud(const std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                          const pcl::PointIndices::Ptr &out_ground_indices,
                          const pcl::PointIndices::Ptr &out_no_ground_indices);

  /*!
   * Removes the points higher than a threshold
   * @param in_cloud_ptr PointCloud to perform Clipping
   * @param in_clip_height Maximum allowed height in the cloud
   * @param out_clipped_cloud_ptr resulting PointCloud with the points removed
   */
  void ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, const double in_clip_height,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
   * in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
   * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
   */
  void ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                            const pcl::PointIndices &in_indices,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr);

  /*!
   * Removes points up to a certain distance in the XY Plane
   * @param in_cloud_ptr Input PointCloud
   * @param in_min_distance Minimum valid distance, points closer than this will be removed.
   * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
   */
  void RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_min_distance,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr);

  void Run( );

 public:
  RayGroundFilter( );
  void estimateGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_cloud);
};

#endif // POINTS_PREPROCESSOR_RAY_GROUND_FILTER_RAY_GROUND_FILTER_H
