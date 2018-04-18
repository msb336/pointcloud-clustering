#include "includes.h"


int main ( )
{

  std::string parameterfile = "../parameters.txt" ;
  std::vector<std::string> parameters = readparameters (parameterfile) ;

  std::string fname         =          parameters[0];
  bool directory            =   stob ( parameters[1] );
  std::string sname         =          parameters[4];
  std::string option        =          parameters[5];
  bool refine               =   stob ( parameters[6] );
  bool fairing              =   stob ( parameters[7] );
  std::string optimization  =          parameters[8];

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

      if ( refine == true || fairing == true )
      {saver << sname << i << ".off" ;}
      else
      {saver << sname << i << ".mesh";}

      loadfile = loader.str() ;
      savefile = saver.str() ;
    }
    else
    { loadfile = fname;   savefile = sname; }

    Polyhedron poly;
    if ( option == "advancedfront" )
    {
      std::cout << "Advanced front method\n\n" ;
      double per = 0 ;
      std::vector<Point_3> points = loadsimple( loadfile );
      Perimeter perimeter(per);
      std::cout << "Doing the advance" << std::endl;
      poly = advancingFrontSurfaceReconstruction( points, perimeter );
    }
    else
    {
      std::cout << "Scale Space Reconstruction" << std::endl;
      std::vector<Point> pp = loadexact( loadfile );
      poly = scaleSpaceReconstruction ( pp );
    }

    if ( refine == true )
    { poly = refinemesh ( poly ); }
    if ( fairing == true )
    { poly = fair ( poly ); }

    if ( optimization == "lloyd")
    {
        C3t3 optimized = lloydOptimization ( poly, 25, 0.1 );

        // Output
        std::ofstream medit_file ( savefile );
        optimized.output_to_medit(medit_file);

    }
    else
    { savemesh ( poly, savefile ); }



  }


  return 0;

}