#include "includes.h"

int main(int argc, char** argv)
{
 
  const char* fname = argv[1];
  const char* s = argv[2];
  const char* op = argv[3];
  std::stringstream option;
  option << op ;

  std::stringstream savefile;
  savefile << s ;

  std::ifstream in(fname);

  if ( option.str() == "advancedfront")
  {

    double per = 0 ;
    std::vector<Point_3> points = loadsimple( fname );
    std::vector<Facet> facets;
    Perimeter perimeter(per);
    advancingFrontSurfaceReconstruction( points, perimeter, savefile.str() );
  }
  else
  {
    std::vector<Point> pp = loadexact( fname );
    Reconstruction rec = scaleSpaceReconstruction(pp, savefile.str());
  }

  return 0;

}