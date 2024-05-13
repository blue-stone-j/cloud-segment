#include "euclidean_cluster.h"

EuclideanCluster::EuclideanCluster( )
{
}
EuclideanCluster::EuclideanCluster(int minNumIn, int maxNumIn, double clusterToleranceIn) :
  minNum(minNumIn), maxNum(maxNumIn), clusterTolerance(clusterToleranceIn)
{
}
EuclideanCluster::~EuclideanCluster( )
{
}

// get index of points and size of this cloud
bool EuclideanCluster::initCompute(const pcl::PointCloud<pcl::PointXYZ> &cloud_in)
{
  if (cloud_in.points.empty( ))
  {
    return false;
  }
  size_pc = cloud_in.points.size( );
  indices.indices.resize(size_pc);
  for (size_t i = 0; i < size_pc; ++i)
  {
    indices.indices[i] = static_cast<int>(i);
  }

  return true;
}

void EuclideanCluster::computeEuclideanCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                               std::vector<pcl::PointIndices> &cluster_indices)
{
  if (!initCompute(cloud_in))
  {
    std::cout << "cloud is empty" << std::endl;
    return;
  }
  if (indices.indices.size( ) != size_pc)
  {
    std::cout << "indices != size_pc" << std::endl;
    return;
  }

  // sorted set to true if the nearest neighbor search results need to be sorted in ascending order based on their distance to the query point
  pcl::search::KdTree<pcl::PointXYZ> tree(false);

  tree.setInputCloud(cloud_in.makeShared( ));

  int nn_start_idx = 0;
  std::vector<bool> processed(size_pc, false); // false means this point hasn't been clustered

  std::vector<int> nn_indices;    // save indedx of a point's neighbors
  std::vector<float> nn_distance; // save distance of a point's neighbors

  for (size_t i = 0; i < size_pc; ++i)
  {
    if (processed[indices.indices[i]]) // if this point has been clustered, skip this point
    {
      continue;
    }

    int sq_idx = 0;              // index in this cluster
    std::vector<int> seed_queue; // points that belong to same cluster with this point
    seed_queue.emplace_back(indices.indices[i]);
    processed[i] = true;

    //  search neighbor for every point in this cluster; add neighbor into this cluster;
    while (sq_idx < static_cast<int>(seed_queue.size( )))
    {
      int ret = tree.radiusSearch(cloud_in.points[seed_queue[sq_idx]], clusterTolerance, nn_indices, nn_distance);
      if (!ret) // if not neighbor
      {
        sq_idx++;
        continue;
      }
      int nn_indices_size = nn_indices.size( ); // num of neighbors
      for (int j = nn_start_idx; j < nn_indices_size; ++j)
      {
        if (nn_indices[j] == -1 || processed[nn_indices[j]]) // or this neighbor has been clustered, skip this neighbor
        {
          continue;
        }
        seed_queue.emplace_back(nn_indices[j]); // add this neighbor into this cluster
        processed[nn_indices[j]] = true;
      }

      sq_idx++;
    }
    int seed_size = seed_queue.size( );

    // store points in this cluster
    if (seed_size >= minNum && seed_size <= maxNum)
    {
      pcl::PointIndices r;
      r.indices.resize(seed_size);
      for (int j = 0; j < seed_size; ++j)
      {
        r.indices[j] = seed_queue[j];
      }
      r.header = cloud_in.header;
      cluster_indices.emplace_back(r);
      //      std::cout << "cluster size: " << seed_size << std::endl;
    }
  }
}
