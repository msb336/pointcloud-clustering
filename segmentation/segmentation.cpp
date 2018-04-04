#include "includes.h"
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
  std::stringstream directory ( argv[1] );
  std::cout << directory.str () << '\n' ;
  std::stringstream fullfile ;
  fullfile << argv[1] << argv[2] ;

  pointCloud::Ptr cloud =  loadcloud ( fullfile.str() );
  float   tolerance        =  atof( argv[3] );
  float   filter_size      =  atof( argv[4] );

  std::stringstream savesegs ( argv[5] ) ;

  if ( filter_size >= 0.001f )
  {
  	downsample ( cloud, filter_size );
  	// visualize ( cloud , true);
    noisefilter ( cloud, 50, 1.0 );
  }
    visualize (cloud, false) ;

  	std::vector<colorCloud::Ptr> segments = euclideanCluster ( cloud, tolerance );
    // std::vector<colorCloud::Ptr> ksegs = kmeans( filtered );
  	std::cout << segments.size ( ) << std::endl;
  	visualize ( segments );

    
    if ( savesegs.str() == "true")
    {
      writetoPCD ( directory.str(), segments ) ;
    }



}
