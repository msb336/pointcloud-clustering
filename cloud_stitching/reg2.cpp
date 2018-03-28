#include "includes.h"


int main()
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

  loadData(data);

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("No Data\n");
  }
  PCL_INFO ("Loaded %d datasets.\n", (int)data.size ());
  
  // Create a PCLVisualizer object
/*   p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2); */

	PointCloud::Ptr result (new PointCloud), source, target;
  PointCloud::Ptr total ( new PointCloud);
      total->width    = 1;
      total->height   = 1;
      total->is_dense = false;
      total->points.resize (total->width * total->height);

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  int stepsize = 50;
  for (size_t i = stepsize; i < data.size (); i+=stepsize)
  {

    source = data[i-stepsize].cloud;
    // cloudfilter (source, 0.01f);
    target = data[i].cloud;
    // cloudfilter (target, 0.01f);

    // Add visualization data
    // showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-stepsize].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    PCL_INFO ("Result size: %d\n", result->points.size());
    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

		//save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
    ss << "../individual clouds/aligned" << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);
    *total += *result;
  }
pcl::io::savePCDFile("../fullconcat.pcd", *total);
    
}