#include "../segmentation/includes.h"

typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::PointNormal> pointNormal;

struct lbeam
{
	float t;
	float l;
	float h;
	float w;

	float t_certainty;
	float l_certainty;
	float h_certainty;
	float w_certainty;
};

lbeam fit_beam ( cloud )


void visualize ( pointCloud::Ptr cloud , Normals::Ptr normals)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "Single Cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.5, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Single Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


int main ( int argc, char** argv)
{
  std::stringstream readfile ( argv[1] );
  std::vector<std::string> parameters = readparameters ( readfile.str() );

  std::string fullfile    	=   parameters[0];
  float radius 			  	=	stof ( parameters[1] );
  float tolerance			=	stof ( parameters[2] );
  int 	minclust 			= 	1;
  pointCloud::Ptr cloud 	= 	loadcloud ( fullfile );
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  Normals::Ptr cloud_normals (new Normals);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch ( radius );

  // Compute the features
  ne.compute (*cloud_normals);

  visualize ( cloud, cloud_normals);
  // Pull plane model, length vector, and first and last point instead of all points
  std::vector<colorCloud::Ptr> segments = sacSegmentation ( cloud, tolerance, minclust, true);

  //Iterate across plane model matching normals to best fit for some cross-section (try it with just an l-beam for now)

  visualize ( segments );
  pointNormal::Ptr cloudNormals ( new pointNormal );
  pcl::concatenateFields(*cloud, *cloud_normals, *cloudNormals);

  return 0;
}
