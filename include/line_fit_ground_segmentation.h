#ifndef LINE_FIT_GROUND_SEGMENTATION_H
#define LINE_FIT_GROUND_SEGMENTATION_H

#include <atomic>
#include <list>
#include <mutex>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 每个bin中的点被压缩为一个点，根据论文，取高度最小的点; 每个点包含高度和距离两个值
class Bin
{
 public:
  /*设定了一个最小Z点的结构体，其中是一个二维结构，在x-y平面的线段长度，以及z轴上的数值*/
  struct MinZPoint
  {
    MinZPoint( ) :
      z(0), d(0) {}
    MinZPoint(const double &d, const double &z) :
      z(z), d(d) {}
    bool operator==(const MinZPoint &comp) { return z == comp.z && d == comp.d; }

    double z;
    double d;
  };

 private:
  std::atomic<double> min_z;
  std::atomic<bool> has_point_;
  std::atomic<double> min_z_range;

 public:
  Bin( );

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin &segment);

  void addPoint(const pcl::PointXYZ &point);

  void addPoint(const double &d, const double &z);

  MinZPoint getMinZPoint( );

  inline bool hasPoint( ) { return has_point_; }
};

// 对应论文中的一个小扇形，因此只需要保存距离和高度即可
class Segment
{
 public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line; // two ends of lines

  typedef std::pair<double, double> LocalLine; // k(slope) and b(interception)

 private:
  // Parameters. Description in GroundSegmentation.
  const double max_slope_;
  const double max_error_;
  const double long_threshold_;
  const double max_long_height_;
  const double max_start_height_;
  const double sensor_height_;

  std::vector<Bin> bins_; // 一个扇形中的所有bin

  std::list<Line> lines_; // 一个扇形中的所有直线

  LocalLine fitLocalLine(const std::list<Bin::MinZPoint> &points);

  double getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line);

  double getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line);

  Line localLineToLine(const LocalLine &local_line, const std::list<Bin::MinZPoint> &line_points);

 public:
  Segment(const unsigned int &n_bins,
          const double &max_slope, const double &max_error,
          const double &long_threshold, const double &max_long_height,
          const double &max_start_height, const double &sensor_height);

  double verticalDistanceToLine(const double &d, const double &z);

  void fitSegmentLines( );

  inline Bin &operator[](const size_t &index)
  {
    return bins_[index];
  }

  inline std::vector<Bin>::iterator begin( )
  {
    return bins_.begin( );
  }

  inline std::vector<Bin>::iterator end( )
  {
    return bins_.end( );
  }

  bool getLines(std::list<Line> *lines);
};

struct GroundSegmentationParams
{
  GroundSegmentationParams( ) :
    r_min_square(0.3 * 0.3),
    r_max_square(20 * 20),
    n_bins(30),
    n_segments(180),
    max_dist_to_line(0.15),
    max_slope(1),
    max_error_square(0.01),
    long_threshold(2.0),
    max_long_height(0.1),
    max_start_height(0.2),
    sensor_height(0.2),
    line_search_angle(0.2),
    n_threads(4) {}

  // Minimum range of segmentation.
  double r_min_square;
  // Maximum range of segmentation.
  double r_max_square;
  // Number of radial bins.
  int n_bins;
  // Number of angular segments.
  int n_segments;
  // Maximum distance to a ground line to be classified as ground.
  double max_dist_to_line;
  // Max slope to be considered ground line.
  double max_slope;
  // Max error for line fit.
  double max_error_square;
  // Distance at which points are considered far from each other.
  double long_threshold;
  // Maximum slope for
  double max_long_height;
  // Maximum heigh of starting line to be labelled ground.
  double max_start_height;
  // Height of sensor above ground.
  double sensor_height;
  // How far to search for a line in angular direction [rad].
  double line_search_angle;
  // Number of threads.
  unsigned int n_threads;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine; // two ends of a line segment

/*从line_fit_ground_segmentation_ros中跳转到这里*/
class GroundSegmentation
{
  const GroundSegmentationParams params_;

  // Access with segments_[segment][bin].
  /*定义了一个划分*/
  std::vector<Segment> segments_;

  // Bin index of every point.
  /*设定了一个针对于所有点而言的bin的索引(segment_index, bin_index)*/
  std::vector<std::pair<int, int>> bin_index_;

  // 2D coordinates (d, z) of every point in its respective segment.
  /*我们设定了一个二维的坐标(d,z)对于各自的分割*/
  std::vector<Bin::MinZPoint> segment_coordinates_;

  /*划分集群*/
  void assignCluster(std::vector<int> *segmentation);

  /*划分集群线程*/
  void assignClusterThread(const unsigned int &start_index,
                           const unsigned int &end_index,
                           std::vector<int> *segmentation);

  /*插入点云; divide points into bins and segments*/
  void insertPoints(const PointCloud &cloud);

  /*插入线程; divide points into bins and segments*/
  void insertionThread(const PointCloud &cloud, const size_t start_index, const size_t end_index);

  /*获得最小的z点*/
  void getMinZPoints(PointCloud *out_cloud);

  /*得到计算出来的线*/
  void getLines(std::list<PointLine> *lines);

  /*直线拟合线程*/
  void lineFitThread(const unsigned int start_index, const unsigned int end_index,
                     std::list<PointLine> *lines, std::mutex *lines_mutex);

  /*将最小z点转化到3D中*/
  pcl::PointXYZ minZPointTo3d(const Bin::MinZPoint &min_z_point, const double &angle);

  /*获取最小z点的线程*/
  void getMinZPointCloud(PointCloud *cloud);


 public:
  GroundSegmentation(const GroundSegmentationParams &params = GroundSegmentationParams( ));

  void estimateGround(const PointCloud &cloud, std::vector<int> *segmentation);
};


#endif