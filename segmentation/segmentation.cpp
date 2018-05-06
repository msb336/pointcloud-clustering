#include "includes.h"
#include <stdlib.h>
#include <stdio.h>

int main ( int argc, char** argv )
{
  std::stringstream readfile ( argv[1] );
  std::vector<std::string> parameters = readparameters ( readfile.str() );

  std::string fullfile    =   parameters[0];

  float sample_leaf_size  =   stof ( parameters[1] );
  bool noise_filter       =   stob ( parameters[2] );
  int noise_neighbors     =   stoi ( parameters[3] );
  float noise_std         =   stof ( parameters[4] );
  
  std::string seg_method  =   parameters[5] ;
  float tolerance         =   stof ( parameters[6] );
  int   minclust          =   stoi ( parameters[7] );
  int   maxclust          =   stoi ( parameters[8] );

  float post_leaf_size    =   stof ( parameters[9] );
  bool post_noise         =   stob ( parameters[10] );
  int post_neighbors      =   stoi ( parameters[11] );
  float post_std          =   stof ( parameters[12] );


  bool savesegs           =   stob ( parameters[13] );
  std::string filetype    =   parameters[15] ;
  bool vis                =   stob ( parameters[16] );

  pointCloud::Ptr cloud = loadcloud ( fullfile );

  if ( sample_leaf_size >= 0.001f )
  	downsample ( cloud, sample_leaf_size );

  if ( noise_filter == true )
    noisefilter ( cloud, noise_neighbors, noise_std );

  if ( vis == true)
    visualize ( cloud, true ) ;
    



  std::vector<colorCloud::Ptr> segments;

  if (seg_method == "sac")
  {
    segments = sacSegmentation ( cloud, tolerance, minclust);
  }
  else if ( seg_method == "euclideancluster" )
  {
    segments = euclideanCluster ( cloud, tolerance, minclust, maxclust );
  }
  else
  {
    std::cerr << "Unrecognized segmentation method: " << seg_method << std::endl;
    exit(1);
    
  }


  if ( post_leaf_size > 0 || post_noise == true )
  {
    for (size_t i = 0; i < segments.size (); i ++ )
    {
      if ( post_leaf_size > 0 )
      {
        downsample ( segments[i], post_leaf_size );
      }
      if ( post_noise == true )
      {
        noisefilter ( segments[i], post_neighbors, post_std );
      }
    }
  }
  
  if ( vis == true )
    visualize ( segments );

  if ( savesegs == true )
  {
    std::transform ( filetype.begin (), filetype.end (), filetype.begin (), ::tolower );

    std::string saveloc = parameters[14] ;

    if ( filetype == ".xyz" || filetype == "xyz")
    {
      writetoXYZ ( saveloc, segments ) ;
    }
    else if ( filetype == ".pcd" || filetype == "pcd" )
    {
      writetoPCD ( saveloc, segments ) ;
    }
    else
    {
      std::cerr << "Unrecognized file format: " << filetype << std::endl;
    }
    
  }


}
