#include "ring_ground_filter.h"

GroundFilter::GroundFilter( )
{
  point_topic_ = "/points_raw";

  floor_removal_ = true;

  sensor_model_ = 64;

  sensor_height_ = 1.80;

  max_slope_ = 10.0;

  vertical_threshold_ = 0.08;


  no_ground_topic = "/points_no_ground";

  ground_topic = "/points_ground";


  int default_horizontal_res;
  switch (sensor_model_)
  {
    case 64:
      default_horizontal_res = 2083;
      break;
    case 32:
      default_horizontal_res = 2250;
      break;
    case 16:
      default_horizontal_res = 1800;
      break;
    default:
      default_horizontal_res = DEFAULT_HOR_RES;
      break;
  }
  horizontal_res_ = default_horizontal_res;


  vertical_res_ = sensor_model_;
  InitLabelArray(sensor_model_);
  InitRadiusTable(sensor_model_);
}

void GroundFilter::InitLabelArray(int in_model)
{
  for (int a = 0; a < vertical_res_; a++)
  {
    class_label_[a] = UNKNOWN;
  }
}


void GroundFilter::InitRadiusTable(int in_model)
{
  double a;     // vertical resolution
  double b;     // max slope
  double theta; // angle between current ray and horizon

  switch (in_model)
  {
    case 64:
      a = 1.0 / 3 * M_PI / 180;
      b = max_slope_ * M_PI / 180;
      for (int i = 0; i < 64; i++)
      {
        if (i <= 31)
        {
          if (i == 31) a = -a;
          theta            = (1.0 / 3 * i - 2.0) * M_PI / 180;
          radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
        }
        else
        {
          a                = 0.5 * M_PI / 180;
          theta            = (8.83 + (0.5) * (i - 32.0)) * M_PI / 180;
          radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
        }
      } // endfor: have traversed every ring
      break;
    case 32:
      a = 4.0 / 3 * M_PI / 180;
      b = max_slope_ * M_PI / 180;
      for (int i = 0; i < 32; i++)
      {
        theta            = (-31.0 / 3 + (4.0 / 3) * i) * 180 / M_PI;
        radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
      }
      break;
    case 16:
      a = 2.0 * M_PI / 180;
      b = max_slope_ * M_PI / 180;
      for (int i = 0; i < 16; i++)
      {
        theta            = (-30.0 / 2 + (2.0) * i) * 180 / M_PI;
        radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
      }
      break;
    default:
      a = 1.0 / 3 * M_PI / 180;
      b = max_slope_ * M_PI / 180;
      for (int i = 0; i < 64; i++)
      {
        if (i <= 31)
        {
          if (i == 31) a = -a;
          theta            = (1.0 / 3 * i - 2.0) * M_PI / 180;
          radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
        }
        else
        {
          a                = 0.5 * M_PI / 180;
          theta            = (8.83 + (0.5) * (i - 32.0)) * M_PI / 180;
          radius_table_[i] = fabs(sensor_height_ * (1.0 / (tan(theta) + tan(b)) - 1.0 / (tan(a + theta) + tan(b))));
        }
      }
      break;
  }
}

void GroundFilter::InitDepthMap(int in_width)
{
  const int mOne = -1;
  index_map_.resize(vertical_res_, vertical_res_);
  index_map_.setConstant(mOne);
}

void GroundFilter::FilterGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &in_cloud_msg,
                                pcl::PointCloud<PointXYZIR> &out_groundless_points,
                                pcl::PointCloud<PointXYZIR> &out_ground_points)
{
  PointXYZIR point;
  InitDepthMap(horizontal_res_);

  for (size_t i = 0; i < in_cloud_msg->points.size( ); i++)
  {
    double u = atan2(in_cloud_msg->points[i].y, in_cloud_msg->points[i].x) * 180 / M_PI;
    if (u < 0) { u = 360 + u; }
    int column              = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
    int row                 = vertical_res_ - 1 - in_cloud_msg->points[i].ring;
    index_map_(row, column) = i;
  }

  for (int i = 0; i < horizontal_res_; i++) // traverse every column
  {
    // e.g. if from distant to close: ground -> car -> ground, this ray will be divided into 3 segments
    Label point_class[vertical_res_];
    int point_index[vertical_res_];
    int point_index_size = 0;
    double z_max         = 0; // z max in this segment
    double z_min         = 0; // z min in this segment
    double r_ref         = 0; // radius of last point; == 0 when first point in this column
    std::copy(class_label_, class_label_ + vertical_res_, point_class);
    for (int j = 0; j < vertical_res_; j++) // up-down
    {
      if (index_map_(j, i) > -1 && point_class[j] == UNKNOWN) // point exist and hasn't been classified
      {
        double x0     = in_cloud_msg->points[index_map_(j, i)].x;
        double y0     = in_cloud_msg->points[index_map_(j, i)].y;
        double z0     = in_cloud_msg->points[index_map_(j, i)].z;
        double r0     = sqrt(x0 * x0 + y0 * y0); // radius of current point
        double r_diff = fabs(r0 - r_ref);
        if (r_diff < radius_table_[j] || r_ref == 0) // vertical || first point in this column
        {
          r_ref = r0;
          if (z0 > z_max || r_ref == 0) z_max = z0;
          if (z0 < z_min || r_ref == 0) z_min = z0;
          point_index[point_index_size] = j;
          point_index_size++;
        }
        else // not first point
        {
          if (point_index_size > 1 && (z_max - z_min) > vertical_threshold_) // this segment is classified as vertical
          {
            for (int m = 0; m < point_index_size; m++)
            {
              int index       = index_map_(point_index[m], i);
              point.x         = in_cloud_msg->points[index].x;
              point.y         = in_cloud_msg->points[index].y;
              point.z         = in_cloud_msg->points[index].z;
              point.intensity = in_cloud_msg->points[index].intensity;
              point.ring      = in_cloud_msg->points[index].ring;
              out_groundless_points.push_back(point);
              point_class[point_index[m]] = VERTICAL;
            }
            point_index_size = 0;
          }
          else
          {
            for (int m = 0; m < point_index_size; m++) // this segment is classified as ground
            {
              int index       = index_map_(point_index[m], i);
              point.x         = in_cloud_msg->points[index].x;
              point.y         = in_cloud_msg->points[index].y;
              point.z         = in_cloud_msg->points[index].z;
              point.intensity = in_cloud_msg->points[index].intensity;
              point.ring      = in_cloud_msg->points[index].ring;
              out_ground_points.push_back(point);
              point_class[point_index[m]] = GROUND;
            }
            point_index_size = 0;
          }
          r_ref                         = r0;
          z_max                         = z0;
          z_min                         = z0;
          point_index[point_index_size] = j;
          point_index_size++;
        }
      }                                                    // have classified this point
      if (j == vertical_res_ - 1 && point_index_size != 0) // last point in this column
      {
        if (point_index_size > 1 && (z_max - z_min) > vertical_threshold_)
        {
          for (int m = 0; m < point_index_size; m++)
          {
            int index       = index_map_(point_index[m], i);
            point.x         = in_cloud_msg->points[index].x;
            point.y         = in_cloud_msg->points[index].y;
            point.z         = in_cloud_msg->points[index].z;
            point.intensity = in_cloud_msg->points[index].intensity;
            point.ring      = in_cloud_msg->points[index].ring;
            out_groundless_points.push_back(point);
            point_class[point_index[m]] = VERTICAL;
          }
          point_index_size = 0;
        }
        else
        {
          for (int m = 0; m < point_index_size; m++)
          {
            int index       = index_map_(point_index[m], i);
            point.x         = in_cloud_msg->points[index].x;
            point.y         = in_cloud_msg->points[index].y;
            point.z         = in_cloud_msg->points[index].z;
            point.intensity = in_cloud_msg->points[index].intensity;
            point.ring      = in_cloud_msg->points[index].ring;
            out_ground_points.push_back(point);
            point_class[point_index[m]] = GROUND;
          }
          point_index_size = 0;
        }
      }
    } // endfor: have traverse all points in this column
  }   // endfor: have traversed all columns
}

void GroundFilter::estimateGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &in_cloud_msg, pcl::PointCloud<PointXYZIR> &ground_points)
{
  pcl::PointCloud<PointXYZIR> vertical_points;

  vertical_points.header = in_cloud_msg->header;
  ground_points.header   = in_cloud_msg->header;
  vertical_points.clear( );
  ground_points.clear( );

  FilterGround(in_cloud_msg, vertical_points, ground_points);

  if (!floor_removal_)
  {
    vertical_points = *in_cloud_msg;
  }
}