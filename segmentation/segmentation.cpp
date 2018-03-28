#include "includes.h"
#include <stdlib.h>
#include <stdio.h>



int main(int argc, char** argv)
{
  pointCloud::Ptr rawcloud =  loadcloud ( argv[1] );
  float   tolerance        =  atof( argv[2] );
  float   filter_size      =  atof( argv[3] );
  if ( filter_size >= 0.001f )
  {
  	pointCloud::Ptr filtered = cloudfilter ( rawcloud, filter_size );
  	visualize ( filtered , true);
  	std::vector<colorCloud::Ptr> segments = euclideanCluster ( filtered, tolerance );
    std::vector<colorCloud::Ptr> ksegs = kmeans( filtered );
  	std::cout << segments.size ( ) << std::endl;
  	visualize ( segments );
  }
  else
  {
  	visualize( rawcloud, true);
  }
}
