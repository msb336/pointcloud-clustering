/////////////////////////////////////
/*
Register and transform a series of point clouds into a conjoined cloud
*/
// Includes
#include <iostream>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

using namespace std;


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


///////////////////////////////////////////////
// Read/write functions
void writetofile(string filename, PointCloud::Ptr cloud)
{
  PointCloud writecloud;
    // Fill in the cloud data
  writecloud.width    = cloud->width;
  writecloud.height   = cloud->height;
  writecloud.is_dense = false;
  writecloud.points.resize (writecloud.width * writecloud.height);

  
    for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    writecloud.points[i].x = cloud->points[i].x;
    writecloud.points[i].y = cloud->points[i].y;
    writecloud.points[i].z = cloud->points[i].z;
  }

  pcl::io::savePCDFileASCII (filename, writecloud);
}

PointCloud::Ptr loadpcd(string filename)
{
  pcl::PCDReader reader;
  PointCloud::Ptr cloud ( new PointCloud () );
  reader.read (filename, *cloud);
  return cloud;
}


/////////////////////////////////////
// Struct Definitions
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;
  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


////////////////////////////////////////////////////////////////////////////////

void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


// Pair Alignment
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, const PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }
  

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
  
  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  
  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    
    reg.align (*reg_result);
    
		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
    
  }
  
	
  // Get the transformation from target to source
  targetToSource = Ti.inverse();
  
  //
  // Transform target back in source frame
  
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  
  //add the source to the transformed target
  // *output += *cloud_src;
  
  final_transform = targetToSource;
  
  
 }





//////////////////////////////
int main (int argc, char** argv)
{
  string loadfile1 = "../barpcdfiles/barframe0.pcd";
  string loadfile2 = "../barpcdfiles/barframe1.pcd";
  
  PointCloud::Ptr cloud1 = loadpcd(loadfile1);
  PointCloud::Ptr cloud2 = loadpcd(loadfile2);
  PointCloud::Ptr temp = cloud1;
  PointCloud::Ptr fullcloud = cloud1;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  for (int i = 0; i<=375; i++)
  {
    stringstream s;
    s << i;
    string loadfile = "../barpcdfiles/barframe" + s.str() + ".pcd";
    cloud1 = cloud2;
    cloud2 = loadpcd(loadfile);

    pairAlign(cloud1, cloud2, temp, GlobalTransform, true);
    

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *fullcloud, GlobalTransform);

    *fullcloud += *temp;
    PCL_INFO ("Temp Cloud Size: %d.       ", temp->points.size());
    PCL_INFO ("Full Cloud Size: %d.\n", fullcloud->points.size());
    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;
  }

  string writefile = "../barframe0-4registered.pcd";

  writetofile(writefile, fullcloud);
  
  return (0);
}