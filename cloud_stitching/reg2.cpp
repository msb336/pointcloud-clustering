#include "includes.h"


int main(int argc, char** argv)
{
  // Load data
  std::stringstream directory ( argv[1] ) ;
  int start = atoi(argv[2]);
  int finish = atoi (argv[3]);

  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

  loadData(directory.str(), start, finish, data);

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("No Data\n");
  }
  PCL_INFO ("Loaded %d datasets.\n", (int)data.size ());
  

	PointCloud::Ptr result (new PointCloud), source, target;
  PointCloud::Ptr total ( new PointCloud);
      total->width    = 1;
      total->height   = 1;
      total->is_dense = false;
      total->points.resize (total->width * total->height);

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  int stepsize = 1;
  int k = 50;
  float std = 1.0;
  for (size_t i = stepsize; i < data.size (); i+=stepsize)
  {
    if ( i == stepsize )
    {
      source = data[i-stepsize].cloud;
      noisefilter ( source, k, std );
    }
    else
    {
      source = target;
    }


    target = data[i].cloud;
    noisefilter ( target, k, std );


    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-stepsize].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    PCL_INFO ("Result size: %d\n", result->points.size());
    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

		//save aligned pair, transformed into the first cloud's frame
        // std::stringstream ss;
    // ss << "../individual clouds/aligned" << i << ".pcd";
    // pcl::io::savePCDFile (ss.str (), *result, true);
    *total += *result;
  }

  std::stringstream path;
  path << directory.str() << "concatenated/" ;
  fs::path dir(path.str());
  if ( fs::create_directory(dir))
  {
  }
  std::stringstream title;
  title << path.str() << "fullconcatenation.pcd" ;
pcl::io::savePCDFile(title.str() , *total);
    
}