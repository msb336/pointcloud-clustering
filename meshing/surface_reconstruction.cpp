#include "includes.h"


int main ( )
{

  std::string parameterfile = "../parameters.txt" ;
  std::vector<std::string> parameters = readparameters (parameterfile) ;

  for (int i = 0; i < parameters.size(); i++)
  {
    std::cout << parameters[i] << std::endl;
  }

  std::string fname = parameters[0];
  std::string savefile = parameters[1];
  std::string option = parameters[2];

  if ( option == "advancedfront" )
  {
    std::cout << "Advanced front method\n\n" ;
    double per = 0 ;
    std::vector<Point_3> points = loadsimple( fname );
    std::vector<Facet> facets;
    Perimeter perimeter(per);
    advancingFrontSurfaceReconstruction( points, perimeter, savefile );
  }
  else
  {
    std::vector<Point> pp = loadexact( fname );
    Reconstruction rec = scaleSpaceReconstruction ( pp, savefile );
  }

  return 0;

}