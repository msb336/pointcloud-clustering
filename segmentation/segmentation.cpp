#include "includes.h"
#include <stdlib.h>
#include <stdio.h>

int main ( )
{
  std::vector<std::string> parameters = readparameters ( "../parameters.txt" );

  std::string fullfile =   parameters[0];

  float sample_leaf_size      =   stof ( parameters[1] );
  bool noise_filter     =   stob ( parameters[2] );
  int noise_neighbors   =   stoi ( parameters[3] );
  float noise_std       =   stof ( parameters[4] );

  float tolerance       =   stof ( parameters[5] );
  int   minclust        =   stoi ( parameters[6] );
  int   maxclust        =   stoi ( parameters[7] );

  float post_leaf_size =   stof ( parameters[8] );
  bool post_noise       =   stob ( parameters[9] );
  int post_neighbors    =   stoi ( parameters[10] );
  float post_std        =   stof ( parameters[11] );


  bool savesegs        =   stob ( parameters[12] );
  std::string filetype =   parameters[14] ;
  bool vis             =   stob ( parameters[15] );

  pointCloud::Ptr cloud = loadcloud ( fullfile );

  if ( sample_leaf_size >= 0.001f )
  	downsample ( cloud, sample_leaf_size );

  if ( noise_filter == true )
    noisefilter ( cloud, noise_neighbors, noise_std );

  if ( vis == true)
    visualize ( cloud, false ) ;

  std::vector<colorCloud::Ptr> segments = euclideanCluster ( cloud, tolerance, minclust, maxclust );

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
    std::string saveloc = parameters[13] ;

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
