
#include <thread>

#include "line_fit_ground_segmentation.h"

/************* Bin *************/

Bin::Bin( ) :
  min_z(std::numeric_limits<double>::max( )),
  has_point_(false)
{}

Bin::Bin(const Bin &bin) :
  min_z(std::numeric_limits<double>::max( )),
  has_point_(false)
{}

// note that d isn't stored in class, we only store min of d
void Bin::addPoint(const pcl::PointXYZ &point)
{
  const double d = sqrt(point.x * point.x + point.y * point.y);
  addPoint(d, point.z);
}

/*添加点*/
void Bin::addPoint(const double &d, const double &z)
{
  has_point_ = true;
  if (z < min_z)
  {
    min_z       = z;
    min_z_range = d;
  }
}

/*判断是否有点，如果有点存在的话，将我们之前算好的min_z和min_z_range传入到point之中*/
Bin::MinZPoint Bin::getMinZPoint( )
{
  MinZPoint point;

  if (has_point_)
  {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}

/************* Segment *************/

Segment::Segment(const unsigned int &n_bins,
                 const double &max_slope, const double &max_error,
                 const double &long_threshold,
                 const double &max_long_height, const double &max_start_height,
                 const double &sensor_height) :

  max_slope_(max_slope),
  max_error_(max_error),
  long_threshold_(long_threshold),
  max_long_height_(max_long_height),
  max_start_height_(max_start_height),
  sensor_height_(sensor_height), bins_(n_bins)
{}

/*分割线拟合*/
void Segment::fitSegmentLines( )
{
  // Find first point.
  auto line_start = bins_.begin( );
  /*在整个的bins中找第一个有点的bin*/
  while (!line_start->hasPoint( ))
  {
    ++line_start;
    // Stop if we reached last point.
    if (line_start == bins_.end( )) { return; }
  }
  // Fill lines.
  bool is_long_line        = false;
  double cur_ground_height = -sensor_height_;
  /*将线的第一个点的信息传递到*/
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint( ));
  LocalLine cur_line = std::make_pair(0, 0);
  /*从第一个点开始对于bins上的每一个点都进行遍历操作;遍历每个bin*/
  for (auto line_iter = line_start + 1; line_iter != bins_.end( ); ++line_iter)
  {
    /*如果我们该bin上有点，则进行后面的操作*/
    if (line_iter->hasPoint( ))
    {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint( );
      /*如果两个的点的距离大于我们设定的阈值，则我们认为这两个点之间是长线*/
      if (cur_point.d - current_line_points.back( ).d > long_threshold_) { is_long_line = true; }
      /*针对于后面几次遍历而言的，至少有三个点的情况下*/
      if (current_line_points.size( ) >= 2)
      {
        // Get expected z value to possibly reject far away points.
        /*获取远离点的z值*/
        double expected_z = std::numeric_limits<double>::max( );
        /*如果是长线段且当前线段的点数大于2，则我们获取到期待的z，这个在第一次迭代的时候不会被执行*/
        if (is_long_line && current_line_points.size( ) > 2)
        {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        /*将当前点插入到current_line之中*/
        current_line_points.push_back(cur_point);
        /*对于当前线点传入到本地线拟合中，得到最后的结果*/
        cur_line = fitLocalLine(current_line_points);
        /*将我们经过本地线拟合之后的*/
        const double error = getMaxError(current_line_points, cur_line);
        // Check if not a good line.
        /*将算出来的误差和最大误差进行比较，判断是否是一个合格的线*/
        // 如果该点不在当前直线上，则另起一条直线
        if (error > max_error_
            || std::fabs(cur_line.first) > max_slope_
            || (is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_))
        {
          // Add line until previous point as ground.
          /*添加线直到浅一点是地面点*/
          current_line_points.pop_back( );
          // Don't let lines with 2 base points through.
          /*不要让有两个基点的线穿过*/
          if (current_line_points.size( ) >= 3)
          {
            /*对于当前线点进行本地拟合，得到一条新的线*/
            const LocalLine new_line = fitLocalLine(current_line_points);
            /*将进行处理后的点放入到线中*/
            lines_.push_back(localLineToLine(new_line, current_line_points));
            /*计算出当前地面点的高度*/
            cur_ground_height = new_line.first * current_line_points.back( ).d + new_line.second;
          }
          // Start new line.
          is_long_line = false;
          /*erase在删除的过程中还是减少vector的size*/
          current_line_points.erase(current_line_points.begin( ), --current_line_points.end( )); // note that last point is reserved
          --line_iter;
        }
        // Good line, continue.
        else
        {
        }
      }
      /*在有1到2个点的情况下的处理*/
      else
      {
        // Not enough points.
        /*判断是否满足添加条件，添加这些点*/
        if (cur_point.d - current_line_points.back( ).d < long_threshold_ && std::fabs(current_line_points.back( ).z - cur_ground_height) < max_start_height_)
        {
          // Add point if valid.
          current_line_points.push_back(cur_point);
        }
        /*开始一条新的线*/
        else
        {
          // Start new line.
          current_line_points.clear( );
          current_line_points.push_back(cur_point);
        }
      }
    } // endif: have processed if haspoint
  }   // endfor: have traversed bins
  // Add last line.
  /*添加最后一条线*/
  if (current_line_points.size( ) > 2)
  {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

/*本地线到线，得到两个点，构建出一条直线*/
Segment::Line Segment::localLineToLine(const LocalLine &local_line, const std::list<Bin::MinZPoint> &line_points)
{
  Line line;
  const double first_d  = line_points.front( ).d;
  const double second_d = line_points.back( ).d;
  /*跟去前面的值来进行local_line的处理*/
  const double first_z  = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;

  line.first.z  = first_z;
  line.first.d  = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

/*到线的垂直距离*/
double Segment::verticalDistanceToLine(const double &d, const double &z)
{
  static const double kMargin = 0.1;
  double distance             = -1;
  for (auto it = lines_.begin( ); it != lines_.end( ); ++it)
  {
    /*这里设定了论文中要求的距离*/
    /*针对于d点，按照设定的余量在距离范围内找到两个点*/
    if (it->first.d - kMargin < d && it->second.d + kMargin > d)
    {
      /*这里是先算出斜率，将传入的两个点传入到直线中，算出一个最近的z值差，也就是垂直的距离*/
      /*算出找到的两个点之间的斜率*/
      const double delta_z    = it->second.z - it->first.z;
      const double delta_d    = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d) / delta_d * delta_z + it->first.z; //(delta_z/delta_d)是一个斜率
      // 算出最终的距离
      distance = std::fabs(z - expected_z);
      /* Note: A point can only match one line in fact. Therefore, I think better to add "return" here to stop
      iteration.
       */
    }
  } // endfor: have traversed points in this line
  return distance;
}

double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line)
{
  double error_sum = 0;
  for (auto it = points.begin( ); it != points.end( ); ++it)
  {
    const double residual = (line.first * it->d + line.second) - it->z;
    error_sum += residual * residual;
  }
  return error_sum / points.size( );
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line)
{
  double max_error = 0;
  for (auto it = points.begin( ); it != points.end( ); ++it)
  {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error    = residual * residual;
    if (error > max_error) { max_error = error; }
  }
  return max_error;
}

/*本地线拟合*/
Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points)
{
  const unsigned int n_points = points.size( );
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (auto iter = points.begin( ); iter != points.end( ); ++iter)
  {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter)    = iter->z;
    ++counter;
  }
  Eigen::VectorXd result = X.colPivHouseholderQr( ).solve(Y); // X*result = Y; y=kx+b
  LocalLine line_result;
  line_result.first  = result(0);
  line_result.second = result(1);
  return line_result;
}

bool Segment::getLines(std::list<Line> *lines)
{
  if (lines_.empty( ))
  {
    return false;
  }
  else
  {
    *lines = lines_;
    return true;
  }
}

/************* GroundSegmentation *************/

/*地面分割的构造函数*/
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams &params) :
  params_(params),
  segments_(params.n_segments, Segment(params.n_bins,
                                       params.max_slope, params.max_error_square,
                                       params.long_threshold, params.max_long_height,
                                       params.max_start_height, params.sensor_height))
{
}

/*地面分割的分割函数*/
void GroundSegmentation::estimateGround(const PointCloud &cloud, std::vector<int> *segmentation)
{
  /*初始化一些比较基础的东西*/
  std::cout << "Segmenting cloud with " << cloud.size( ) << " points...\n";
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now( );
  segmentation->clear( );
  segmentation->resize(cloud.size( ), 0);
  bin_index_.resize(cloud.size( ));
  segment_coordinates_.resize(cloud.size( ));

  /*插入点云数据; divide points into bins and segments*/
  insertPoints(cloud);
  /*定义地面点的线的参数*/
  std::list<PointLine> lines;

  /*对于传入的分割进行细分; 从这里可以看到对于点云属于障碍物还是属于地面点进行了标签的划分*/
  assignCluster(segmentation);

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now( );
  std::chrono::duration<double, std::milli> fp_ms    = end - start;
  std::cout << "Done! Took " << fp_ms.count( ) << "ms\n";
}

/*获取到线*/
void GroundSegmentation::getLines(std::list<PointLine> *lines)
{
  std::mutex line_mutex;
  std::vector<std::thread> thread_vec(params_.n_threads);
  unsigned int i;
  for (i = 0; i < params_.n_threads; ++i)
  {
    const unsigned int start_index = params_.n_segments / params_.n_threads * i;
    const unsigned int end_index   = params_.n_segments / params_.n_threads * (i + 1);

    thread_vec[i] = std::thread(&GroundSegmentation::lineFitThread, this, start_index, end_index, lines, &line_mutex);
  }
  for (auto it = thread_vec.begin( ); it != thread_vec.end( ); ++it)
  {
    it->join( );
  }
}

/*这里是获取线的操作*/
void GroundSegmentation::lineFitThread(const unsigned int start_index, const unsigned int end_index,
                                       std::list<PointLine> *lines, std::mutex *lines_mutex)
{
  const bool visualize  = lines;
  const double seg_step = 2 * M_PI / params_.n_segments;
  double angle          = -M_PI + seg_step / 2 + seg_step * start_index;
  for (unsigned int i = start_index; i < end_index; ++i)
  {
    segments_[i].fitSegmentLines( );
    // Convert lines to 3d if we want to.
    /*这里也是可视化的一些操作*/
    if (visualize)
    {
      std::list<Segment::Line> segment_lines;
      segments_[i].getLines(&segment_lines);
      for (auto line_iter = segment_lines.begin( ); line_iter != segment_lines.end( ); ++line_iter)
      {
        const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);
        const pcl::PointXYZ end   = minZPointTo3d(line_iter->second, angle);
        lines_mutex->lock( );
        lines->emplace_back(start, end);
        lines_mutex->unlock( );
      }

      angle += seg_step;
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud *cloud)
{
  const double seg_step = 2 * M_PI / params_.n_segments;
  double angle          = -M_PI + seg_step / 2;
  for (auto seg_iter = segments_.begin( ); seg_iter != segments_.end( ); ++seg_iter)
  {
    for (auto bin_iter = seg_iter->begin( ); bin_iter != seg_iter->end( ); ++bin_iter)
    {
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint( ), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

/*根据传入的二维点，也可以通过算出的angle将二维点转化为三维点，主要是x-y平面内的变换*/
pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle)
{
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

/*分配集群，将传入的分割进行簇的划分*/
void GroundSegmentation::assignCluster(std::vector<int> *segmentation)
{
  std::vector<std::thread> thread_vec(params_.n_threads);
  const size_t cloud_size = segmentation->size( );
  for (unsigned int i = 0; i < params_.n_threads; ++i)
  {
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index   = cloud_size / params_.n_threads * (i + 1);
    thread_vec[i]                  = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }
  for (auto it = thread_vec.begin( ); it != thread_vec.end( ); ++it)
  {
    it->join( );
  }
}

/*执行分配集群的线程操作*/
void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation)
{
  /*通过传入的segments的个数，得到划分segment的步长，也就是δ值*/
  const double segment_step = 2 * M_PI / params_.n_segments;
  /*进行遍历操作*/
  /*对于每一个点进行处理，根据距离判断是否属于远离地面的点*/
  for (unsigned int i = start_index; i < end_index; ++i)
  {
    /*首先，得到每一个三维点的二维映射*/
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    /*得到第i个bin的第一个值，作为分割的索引*/
    const int segment_index = bin_index_[i].first;
    /*判定分割的index是需要大于0的*/
    if (segment_index >= 0)
    {
      /*计算处两个点到线的垂直距离*/
      /*将点投影到直线上*/
      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      // Search neighboring segments.
      /*搜索相邻的分割*/
      int steps = 1;
      /*根据划分好的segment来不断的进行数据的处理; 在设定的步长的情况下在搜索框内能有几个步长*/
      /*dist==-1说明在之前步骤中没有找到该点对应的直线，在相邻的seg中寻找对应的直线*/
      while (dist == -1 && steps * segment_step < params_.line_search_angle)
      {
        // Fix indices that are out of bounds.
        /*修复超出范围的索引：确保索引在0~n_segments*/
        int index_1 = segment_index + steps; // 进行正向步长的搜索
        while (index_1 >= params_.n_segments) { index_1 -= params_.n_segments; }
        int index_2 = segment_index - steps; // 进行反向步长的索引
        while (index_2 < 0) { index_2 += params_.n_segments; }
        // Get distance to neighboring lines.
        /*算出根据正反双向最近搜索的点到线段投影的距离*/
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
        // Select larger distance if both segments return a valid distance.
        /*如果两个分割都返回有效距离，则选择更大的距离。*/
        /*只要找到匹配的直线, 距离就会大于零, 达到跳出的条件, 可以直接break. */
        /* 这里实际上优先使用了垂直距离更远的匹配, 我觉得是有问题的, 应使用更近的匹配*/
        /* dist = dist1>dist ? dist1 : dist; dist = dist2>dist ? dist2 : dist; */
        if (dist_1 > dist)
        {
          dist = dist_1;
        }
        if (dist_2 > dist)
        {
          dist = dist_2;
        }
        /*不断增加步长，一直持续下去，直到跳出循环，这样可以保证比较公平的遍历到里面的点*/
        ++steps;
      } // endwhile: 在一定范围内找到了匹配的直线或者没找到。
      /*这里是进行标签的设定*/
      if (dist < params_.max_dist_to_line && dist != -1)
      {
        /*这里是对于是否属于地面点的判定: 1为地面*/
        segmentation->at(i) = 1;
      }
    } // endif: have processed this bin
  }   // endfor: have processed every bin
}

/*获取最小z点的点云数据*/
void GroundSegmentation::getMinZPoints(PointCloud *out_cloud)
{
  /*得到分割的步长，以及bins的步长*/
  const double seg_step = 2 * M_PI / params_.n_segments;
  // const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;
  /*得到最小的r*/
  // const double r_min = sqrt(params_.r_min_square);
  double angle = -M_PI + seg_step / 2;
  for (auto seg_iter = segments_.begin( ); seg_iter != segments_.end( ); ++seg_iter)
  {
    // double dist = r_min + bin_step / 2;
    for (auto bin_iter = seg_iter->begin( ); bin_iter != seg_iter->end( ); ++bin_iter)
    {
      pcl::PointXYZ point;
      if (bin_iter->hasPoint( ))
      {
        /*对于bin_iter进行最小z点的提取*/
        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint( ));
        point.x = cos(angle) * min_z_point.d;
        point.y = sin(angle) * min_z_point.d;
        point.z = min_z_point.z;
        /*将point放入到out_cloud之中*/
        out_cloud->push_back(point);
      }
      /*按照步长增加dist*/
      // dist += bin_step;
    } // endfor: bins
    /*按照划分的步长进行角度的增加*/
    angle += seg_step;
  } // endfor: segments
}

/*插入点云*/
void GroundSegmentation::insertPoints(const PointCloud &cloud)
{
  std::vector<std::thread> threads(params_.n_threads);
  const size_t points_per_thread = cloud.size( ) / params_.n_threads;
  // Launch threads.
  /*根据我们设定的数目来将整个的点云分为几个部分开始处理，利用多线程来处理*/
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i)
  {
    const size_t start_index = i * points_per_thread;
    const size_t end_index   = (i + 1) * points_per_thread - 1;

    threads[i] = std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);
  }
  // Launch last thread which might have more points than others.
  /*启动最后一个可能含有更多点云的线程*/
  const size_t start_index       = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index         = cloud.size( ) - 1;
  threads[params_.n_threads - 1] = std::thread(&GroundSegmentation::insertionThread, this, cloud,
                                               start_index, end_index);
  // Wait for threads to finish.
  for (auto it = threads.begin( ); it != threads.end( ); ++it)
  {
    it->join( );
  }
}

/*线程启动中会执行的函数*/
void GroundSegmentation::insertionThread(const PointCloud &cloud, const size_t start_index, const size_t end_index)
{
  /*同样的先算步长，然后再进行其他的操作*/
  // angel of sector
  const double segment_step = 2 * M_PI / params_.n_segments;
  // range diff
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;
  // min range, where bins start
  const double r_min = sqrt(params_.r_min_square);
  /*对于起始索引和终止索引进行遍历*/
  for (unsigned int i = start_index; i < end_index; ++i)
  {
    pcl::PointXYZ point(cloud[i]);
    /*这里是算模长*/
    const double range_square = point.x * point.x + point.y * point.y;
    const double range        = sqrt(range_square);
    /*判断模长是否属于最小值和最大值之间*/
    if (range_square < params_.r_max_square && range_square > params_.r_min_square)
    {
      /*得到角度*/
      const double angle = std::atan2(point.y, point.x);
      /*根据模场和角度来算出bin的索引以及划分的索引;segment_index的结果可能为180，从而超出索引导致程序崩溃*/
      const unsigned int bin_index     = (range - r_min) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      /*对于设定的属于bin和划分中的集合进行数据的添加*/
      segments_[segment_index][bin_index].addPoint(range, point.z);
      /*将后面的数据作为一个元组全部传递到bin_index之中*/
      bin_index_[i] = std::make_pair(segment_index, bin_index);
    }
    else
    {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);
    }
    /*获取到划分坐标为最小z点的坐标和range*/
    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);
  } // endfor: 完成对起始索引至终止索引的遍历
}
