#include "includes.h"


int main ( )
{
  std::cout << CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl;


  std::string parameterfile = "../parameters.txt" ;
  std::vector<std::string> parameters = readparameters (parameterfile) ;

  std::string fname         =          parameters[0];
  bool directory            =   stob ( parameters[1] );
  std::string sname         =          parameters[4];
  std::string option        =          parameters[5];

  bool  hole_fill           =   stob ( parameters[6] );
  bool  refine              =   stob ( parameters[7] );

  std::vector<float> mesh_criteria;

  for (int i = 8; i <= 12; i++)
  {
    mesh_criteria.push_back ( stof ( parameters[i]));
    std::cout << parameters[i] << std::endl;
  }

  float densityControl      =   stof ( parameters[13] );
  bool fairing              =   stob ( parameters[14] );
  
  std::vector<bool> mesh_operations;
  std::vector<float> mesh_time_limits;
  for ( int j = 15; j <=21; j+=2)
  {
    mesh_operations.push_back ( stob ( parameters[j] ) );
    mesh_time_limits.push_back (stof( parameters[j+1]));
  }



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
    std::stringstream saver ;

    if ( directory == true )
    {
      std::stringstream loader ;


      loader << fname << i <<  ".xyz" ;
      saver << sname << i ;

      loadfile = loader.str() ;
      savefile = saver.str() ;
    }
    else
    { loadfile = fname; saver << sname; }


    Polyhedron poly;
    if ( option == "advancedfront" )
    {
      saver << "Advancedfront";
      std::cout << "Advanced front method\n\n" ;
      double per = 0 ;
      std::vector<Point_3> points = loadsimple( loadfile );
      Perimeter perimeter(per);
      std::cout << "Doing the advance" << std::endl;
      poly = advancingFrontSurfaceReconstruction( points, perimeter );
    }
    else
    {
      saver << "Scalespace";
      std::cout << "Scale Space Reconstruction" << std::endl;
      std::vector<Point> pp = loadexact( loadfile );
      poly = scaleSpaceReconstruction ( pp );
    }
    savemesh( poly, "testmesh.off" );
    if ( hole_fill == true )
    { 
      saver << "Hole_fill" << densityControl;
      fillholes ( poly , densityControl );  
      std::cout << "Poly size after hole fill: " << poly.size_of_facets() << " " << poly.size_of_vertices() << std::endl;
    }

    if ( refine == true )
    { saver << "Refine" << densityControl ;
      refinemesh ( poly, densityControl ); }
    if ( fairing == true )
    { saver << "Fair";
      fair ( poly ); }

    C3t3 optimized = highLevelMesh ( poly, 
                                     mesh_criteria, 
                                     mesh_operations, 
                                     mesh_time_limits, 
                                     saver 
                                    );

    // Output
    std::cout << "Saving to .off file" << std::endl;
    saver << ".off" ;
    std::ofstream medit_file ( saver.str() );

    optimized.output_boundary_to_off ( medit_file );


  }


  return 0;

}