#include "includes.h"


int main ( )
{

  std::string parameterfile = "../parameters.txt" ;
  std::vector<std::string> parameters = readparameters (parameterfile) ;

  std::string fname     =          parameters[0];
  bool directory        =   stob ( parameters[1] );
  std::string sname     =          parameters[4];
  std::string option    =          parameters[5];
  bool refine           =   stob ( parameters[6] );
  bool fairing          =   stob ( parameters[7] );

  int start = 0;        int finish = 0;
  if ( directory == true )
  {
    start               =   stoi ( parameters[2] );
    finish              =   stoi ( parameters[3] );
  }


  for ( int i = start; i <= finish; i++ )
  {
    std::string loadfile ;
    std::string savefile ;
    if ( directory == true )
    {
      std::stringstream loader ;
      std::stringstream saver ;
      loader << fname << i <<  ".xyz" ;
      saver << sname << i << ".off" ;
      loadfile = loader.str() ;
      savefile = saver.str() ;
    }
    else
    {
      loadfile = fname;
      savefile = sname;
    }

    if ( option == "advancedfront" )
    {
      std::cout << "Advanced front method\n\n" ;
      double per = 0 ;
      std::vector<Point_3> points = loadsimple( loadfile );
      std::vector<Facet> facets;
      Perimeter perimeter(per);
      Polyhedron poly = advancingFrontSurfaceReconstruction( points, perimeter, savefile );
    }
    else
    {
      std::vector<Point> pp = loadexact( loadfile );
      Reconstruction rec = scaleSpaceReconstruction ( pp, savefile );
      createPolyhedron ( rec );
    }
    if ( refine == true )
    {
    }
  }

  return 0;

}