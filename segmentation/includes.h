#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/filesystem.hpp>
// #include <pcl/ml/kmeans.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>


#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 


typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
namespace fs = ::boost::filesystem;

pointCloud::Ptr loadcloud(char* loadfile)
{
    // std::string filename = loadfile[1];
    std::string filename = loadfile;
    pointCloud::Ptr newcloud ( new pointCloud );

    pcl::PCDReader reader;
    reader.read (filename, *newcloud);
    return newcloud;
}

void writetoPCD(std::stringstream ss, pointCloud::Ptr cloud)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (ss.str (), *cloud, false);
}

void writetoPCD(std::stringstream ss, colorCloud::Ptr cloud)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (ss.str (), *cloud, false);
}

int32_t castColor(std::vector<int> colorvec)
{
  // PCL rgb color scheme is an int32 with three embedded ints valued 0-255
  int32_t color = (colorvec[0] << 16) | (colorvec[1] << 8) | colorvec[2];
  return color;
}
int32_t castColor(int r, int g, int b)
{
  int32_t color = (r << 16) | (g << 8) | b;
  return color;
}

bool belong(std::vector<int> b1, int b2)
{
  // Basic checker to see if int b2 is inside the vector<int> b1
  bool checker = false;
  for (int i =0; i < b1.size(); i++)
  {
    if (b2 == i)
    {
      checker = true;
      break;
    }
  }
  return checker;
}

void getfilenames(const fs::path& root, const std::string& ext, std::vector<fs::path>& ret)
{
  // Pulls all filenames in directory "root" with extensions "ext" and places them into the vector of fs::paths ret

    if(!fs::exists(root) || !fs::is_directory(root)) return;

    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ext) 
        {
          ret.push_back(it->path().filename());
        }
        ++it;
    }

}

pointCloud::Ptr concatenate(fs::path directory, bool downsample = false)
{
  // Concatenate all pcd files in the specified directory "Directory"

  pointCloud::Ptr concloud (new pointCloud);
  std::vector<fs::path> ret;
  std::string ext = ".pcd";
  // getfilenames(directory, ext, ret);

  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  for (int i = 2; i <=374; i+=2)
  {
    pointCloud::Ptr newcloud (new pointCloud);
    std::stringstream readfile;
    readfile << "../../alignedClouds/concatenated" << i << ".pcd";
    reader.read (readfile.str(), *newcloud);

    *concloud += *newcloud;
    
  }
  std::stringstream writefile;
  writefile << "../../" << "fullcatenated.pcd";
  writer.write<pcl::PointXYZ> (writefile.str (), *concloud, false);
  return concloud;
}

/*
std::vector<colorCloud::Ptr> split(pointCloud::Ptr cloud, pcl::Kmeans::PointsToClusters index)
{
  //Separate pointCloud cloud into individual clouds based on kmeans cluster index
  std::vector<colorCloud::Ptr> segmented_clouds;
  std::vector<int> counter;
  std::vector<std::vector< int > > colors;
  std::vector<int> list;
  int val = 0;
  
  for (int i = 0; i < index.size(); i++)
  {
    bool check = belong(list, index[i]);
    
    if (check == false)
    {
      list.push_back(index[i]);
      colorCloud::Ptr newcloud (new colorCloud);
      std::vector<int> newcolors;
      newcolors.push_back(val);
      newcolors.push_back(val / 5);
      newcolors.push_back(255 - val);
      val += 25;

      newcloud->width    = 1;
      newcloud->height   = 1;
      newcloud->is_dense = false;
      newcloud->points.resize (newcloud->width * newcloud->height);
      newcloud->points[0].x = cloud->points[i].x;
      newcloud->points[0].y = cloud->points[i].y;
      newcloud->points[0].z = cloud->points[i].z;
      newcloud->points[0].rgb = castColor(newcolors);

      segmented_clouds.push_back (newcloud);
      counter.push_back (0);
      colors.push_back(newcolors);
    }
    else
    {
      counter[index[i]]++;
      segmented_clouds[index[i]]->width++;
      segmented_clouds[index[i]]->points.resize (segmented_clouds[index[i]]->width * segmented_clouds[index[i]]->height);
      segmented_clouds[index[i]]->points[counter[index[i]]].x = cloud->points[i].x;
      segmented_clouds[index[i]]->points[counter[index[i]]].y = cloud->points[i].y;
      segmented_clouds[index[i]]->points[counter[index[i]]].z = cloud->points[i].z;
      segmented_clouds[index[i]]->points[counter[index[i]]].rgb = castColor(colors[index[i]]);
    }
  }
  return segmented_clouds;
}
*/

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

void visualize (std::vector<colorCloud::Ptr> segmented_clouds)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  for (int numclouds = 0; numclouds < segmented_clouds.size(); numclouds++)
  {
    std:: stringstream ss;
    ss << numclouds;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segmented_clouds[numclouds]);
    viewer->addPointCloud<pcl::PointXYZRGB> (segmented_clouds[numclouds], rgb, ss.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
  }

  
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void visualize ( pointCloud::Ptr cloud , bool height = false )
{
  if ( height == false )
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "Single Cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Single Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }
  else
  {
    colorCloud::Ptr newcloud ( new colorCloud );
    newcloud->width = cloud->width;
    newcloud->height = cloud->height;
    newcloud->is_dense = true;
    newcloud->points.resize( newcloud->width*newcloud->height );
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);
    // alter color via height value
    for (int j = 0; j < cloud->points.size(); j++)
    {

      newcloud->points[j].x = cloud->points[j].x;
      newcloud->points[j].y = cloud->points[j].y;
      newcloud->points[j].z = cloud->points[j].z;
      int intensity = 255 * (newcloud->points[j].z - minPt.z) / (maxPt.z - minPt.z);

      int32_t color = castColor ( intensity, 50, 255-intensity);
      newcloud->points[j].rgb = (color);

    }
    std::vector<colorCloud::Ptr> singlecolored;
    singlecolored.push_back ( newcloud );

    visualize( singlecolored );
  }


}

 
pointCloud::Ptr cloudfilter(pointCloud::Ptr cloud, float leafsize)
{
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pointCloud::Ptr cloud_filtered (new pointCloud);
  vg.setInputCloud (cloud);
  // vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setLeafSize(leafsize, leafsize, leafsize);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  return cloud_filtered;
}

pcl::SACSegmentation<pcl::PointXYZ> buildsegmentationobject()
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  return seg;
}

/*
std::vector<colorCloud::Ptr> kmeans(pointCloud::Ptr cloud, int segsize)
{

  pcl::Kmeans real(static_cast<int> (cloud->points.size()), 3);
  real.setClusterSize(segsize);

  for (size_t i =0; i < cloud->points.size(); i++)
  {
    std::vector<float> data(3);
    data[0] = cloud->points[i].x;
    data[1] = cloud->points[i].y;
    data[2] = cloud->points[i].z;
    real.addDataPoint(data);
  }
  real.kMeans();

  pcl::Kmeans::Centroids centroids = real.get_centroids();
  pcl::Kmeans::PointsToClusters cluster_index = real.get_index();
  std::vector<colorCloud::Ptr> segments =  split(cloud, cluster_index);
  return segments;
}


std::vector<colorCloud::Ptr> conditionalEuclidean(pointCloud::Ptr cloud, float tolerance)
{
 pcl::ConditionalEuclideanClustering<pcl::PointXYZ> ec ;
 ec.setInputCloud (cloud) ;

}
*/


std::vector<colorCloud::Ptr>  euclideanCluster(pointCloud::Ptr cloud, float tolerance)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_index;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_index);


  std::vector<colorCloud::Ptr> segments =  split(cloud, cluster_index);
  return segments;


}