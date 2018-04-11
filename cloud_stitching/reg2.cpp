#include "includes.h"

int main ( )
{

  // Load data
  std::vector<std::string> parameters = readparameters("../parameters.txt");

  std::string directory   =   parameters[0];
  std::string prefix      =   parameters[1];
  int start               =   stoi ( parameters[2] );
  int finish              =   stoi ( parameters[3] );
  int stepsize            =   stoi ( parameters[4] );
  bool noise_filter       =   stob ( parameters[5] );
  int k                   =   stoi ( parameters[6] );
  float std               =   stof ( parameters[7] );
  float leafsize          =   stof ( parameters[8] );
  std::string saveloc     =   parameters[9];

  std::cout << saveloc << std::endl;


  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

  loadData(directory, prefix, start, finish, stepsize, data);

  // Check user input
  if ( data.empty () )
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

  for (size_t i = 1; i < data.size (); i++)
  {
    if ( i == 1 )
    {
      source = data[i-1].cloud;
      if ( noise_filter == true )
        noisefilter ( source, k, std );
    }
    else
    {
      source = target;
    }


    target = data[i].cloud;
    if ( noise_filter )
      noisefilter ( target, k, std );


    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    PCL_INFO ("Result size: %d\n", result->points.size());
    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

    *total += *result;
  }

  std::stringstream path;
  path << saveloc << "concat" << start << "-" << finish << "step" << stepsize;
  if ( noise_filter == true )
    path << "filtk" << k << "std" << std;
  fs::path dir ( saveloc );
  if ( fs::create_directory(dir))
  {
  }
  pcl::io::savePCDFile(path.str() , *total);

  if ( leafsize > 0)
  {
    downsample(total, leafsize) ;
  }
  path << "downsampled" << leafsize ;
  pcl::io::savePCDFile ( path.str(), *total );
}