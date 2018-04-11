#include <iostream>
#include <vector>
#include <dlib/clustering.h>
#include <dlib/matrix.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>



typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> colorCloud;

using namespace std;
using namespace dlib;

//////////////// PCL ///////////////////
std::vector<colorCloud::Ptr> split(pointCloud::Ptr cloud,  std::vector<pcl::PointIndices> cluster_indices)
{
  //Segment cloud into individual clusters based on point indices
  std::vector<colorCloud::Ptr> segmented_clouds;
  std::vector<int> counter;
  std::vector<std::vector<int> > colors;

  int val = 0;
  int j = 0;


  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    std::vector<int> newcolors;
    newcolors.push_back(val);
    newcolors.push_back(val / 5);
    newcolors.push_back(255 - val);

    uint32_t color = castColor(newcolors);

    val += 255 / cluster_indices.size();

    colorCloud::Ptr cloud_cluster (new colorCloud);
    cloud_cluster->width = it->indices.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->points.resize (cloud_cluster->width * cloud_cluster->height);


    for (int j = 0; j < it->indices.size(); j++)
    {
      cloud_cluster->points[j].x = cloud->points[it->indices[j]].x;
      cloud_cluster->points[j].y = cloud->points[it->indices[j]].y;
      cloud_cluster->points[j].z = cloud->points[it->indices[j]].z;
      cloud_cluster->points[j].rgb = (color);

    }

    segmented_clouds.push_back(cloud_cluster);
  }

  return segmented_clouds;

}

std::vector<colorCloud::Ptr>  euclideanCluster(pointCloud::Ptr cloud, float tolerance, int minclust, int maxclust)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_index;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (minclust);
  ec.setMaxClusterSize (maxclust);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_index);

  std::vector<colorCloud::Ptr> segments =  split(cloud, cluster_index);
  return segments;

}

std::vector<colorCloud::Ptr> sacSegmentation ( pointCloud::Ptr cloud,  float distance, int mininum )
{

  std::vector<pcl::PointIndices> plane_indices;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold ( distance );

  int r       =   255      ;
  int b       =   0        ;
  int in_size =   2000000  ;

  std::cout << "Max size: " << in_size << std::endl;
  std::vector< colorCloud::Ptr > segments ;

  while ( in_size > mininum )
  {
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );

    seg.setInputCloud ( cloud );
    seg.segment ( *inliers, *coefficients );

    extract.setInputCloud ( cloud );
    extract.setIndices ( inliers );
    extract.setNegative ( true );
    extract.filter ( *cloud );

    plane_indices.push_back ( *inliers );

    colorCloud::Ptr plane ( new colorCloud );
    plane->width = inliers->indices.size();
    plane->height = 1;
    plane->is_dense = true;
    plane->points.resize (plane->width * plane->height);

    int32_t color = castColor (r, 0, b);

    r-=25;
    b+=25;
    coefficients->values.resize (4) ;
    in_size = inliers->indices.size () ;
    std::cout << "Plane size: " << inliers->indices.size () << std::endl;
    std::cout << " Plane Coefficients: " << coefficients->values[0] << " " 
    << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    for (size_t j = 0; j < inliers->indices.size(); j++ )
    {
      plane->points[j].x = cloud->points[inliers->indices[j]].x;
      plane->points[j].y = cloud->points[inliers->indices[j]].y;
      plane->points[j].z = cloud->points[inliers->indices[j]].z;
      plane->points[j].rgb = color ;
    }

    segments.push_back ( plane ) ;

  }

  return segments;
}

//////////////// CGAL /////////////////
std::vector<pcl::PointIndices> kmeans ( std::vector<pointMatrix> samples, int num_clusters )
{

    
    typedef radial_basis_kernel<pointMatrix> kernel_type;
 
    kcentroid<kernel_type> kc(kernel_type(0.1),0.01, 8);
    kkmeans<kernel_type> test(kc);

    std::vector<pointMatrix> initial_centers;

   cout << "Number of points: " << samples.size() << endl;

    test.set_number_of_centers(num_clusters);

    pick_initial_centers(num_clusters, initial_centers, samples, test.get_kernel());
  
    test.train(samples,initial_centers);

    std::vector<pcl::PointIndices> cluster_index;
    std::vector<unsigned long> assignments = spectral_cluster(kernel_type(0.1), samples, num_clusters);
    for ( size_t list = 0; list < num_clusters; list++ )
    {
        pcl::PointIndices index;
        cluster_index.push_back ( index );
    }

    for (size_t i = 0; i < assignments.size (); i ++ )
    {
        cluster_index[assignments[i]].indices.push_back ( i );
    }
    
    return cluster_index;
}