#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <map>

#define BOOST_PARAMETER_MAX_ARITY 12

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/tuple.h>
#include <boost/lexical_cast.hpp>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Timer.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>

#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>

#include <CGAL/Surface_mesh.h>

#define CGAL_MESH_3_VERBOSE 1

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_map/property_map.hpp>

namespace fs = ::boost::filesystem;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, Kernel> Mesh_domain;

typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;

#ifdef CGAL_CONCURRENT_MESH_3
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain,CGAL::Default,Concurrency_tag>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;
// Mesh Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;
using namespace CGAL::parameters;


typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3  Point_3;
typedef CGAL::cpp11::array<std::size_t,3> Facet;
typedef K::Vector_3 Vector_3;
typedef std::pair<Point_3, Vector_3> Pwn;

typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>    Reconstruction;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Reconstruction::Facet_const_iterator                   Facet_iterator;
typedef Reconstruction::Point_const_iterator Point_iterator;



namespace std {
  std::ostream&
  operator<<(std::ostream& os, const Facet& f)
  {
    os << "3 " << f[0] << " " << f[1] << " " << f[2];
    return os;
  }
}

struct Perimeter {
  double bound;
  Perimeter(double bound)
    : bound(bound)
  {}
  template <typename AdvancingFront, typename Cell_handle>
  double operator() (const AdvancingFront& adv, Cell_handle& c,
                     const int& index) const
  {
    // bound == 0 is better than bound < infinity
    // as it avoids the distance computations
    if(bound == 0){
      return adv.smallest_radius_delaunay_sphere (c, index);
    }
    // If perimeter > bound, return infinity so that facet is not used
    double d  = 0;
    d = sqrt(squared_distance(c->vertex((index+1)%4)->point(),
                              c->vertex((index+2)%4)->point()));
    if(d>bound) return adv.infinity();
    d += sqrt(squared_distance(c->vertex((index+2)%4)->point(),
                               c->vertex((index+3)%4)->point()));
    if(d>bound) return adv.infinity();
    d += sqrt(squared_distance(c->vertex((index+1)%4)->point(),
                               c->vertex((index+3)%4)->point()));
    if(d>bound) return adv.infinity();
    // Otherwise, return usual priority value: smallest radius of
    // delaunay sphere
    return adv.smallest_radius_delaunay_sphere (c, index);
  }
};

//////////////// I/O ////////////////////
std::vector<std::string> readparameters ( std::string filename )
{
    std::vector<std::string> parameters ;
    std::ifstream inFile;
    char x;
    
    inFile.open(filename);
    if (!inFile) {
        std::cerr << "Unable to open parameter file";
        exit(1);
    }

    while (inFile >> x) 
    {
        std::string sum;
        if ( x == ':')
        {

            while ( inFile.peek() != '\n' && inFile >> x )
            {
                sum += x;
            }

            parameters.push_back (sum);
        }
    }

    inFile.close();
    return parameters ;

}

void savemesh ( Polyhedron poly, std::string savefile )
{
  std::cout << "Saving to: " << savefile << std::endl;;
  std::ofstream out(savefile);
  out.precision(17);
  out << poly << std::endl;
  out.close();
}

void createpath(std::string fullfile)
{
  std::string delimiter = "/";
  std::vector<std::string> tokens;
  size_t pos = 0;


  std::string path = fullfile.substr(0, fullfile.find_last_of("\\/"));

  fs::path dir(path);
  if ( fs::create_directory(dir))
  {
  }

}

bool stob ( std::string truefalse )
{
  if ( truefalse == "true" || truefalse == "1")
  {
    return true;
  }
  else
  {
    return false;
  }
}


std::vector<Point> loadexact ( std::string fname )
{
  std::ifstream in ( fname ) ;
  std::vector<Point> pointset;

   std::copy(std::istream_iterator<Point>(in),
            std::istream_iterator<Point>(),
            std::back_inserter(pointset));
  std::cout << "loaded pointset of size: " << pointset.size() << std::endl;
  return pointset;
}

std::vector<Point_3> loadsimple ( std::string fname )
{
  std::ifstream in ( fname ) ;
  std::vector<Point_3> pointset;

   std::copy(std::istream_iterator<Point_3>(in),
            std::istream_iterator<Point_3>(),
            std::back_inserter(pointset));
  std::cout << "loaded pointset of size: " << pointset.size() << " from file : " 
  << fname
  << std::endl;
  return pointset;
}

Polyhedron offToPoly ( const char* filename )
{
  std::ifstream input(filename);
  Polyhedron poly;
  std::cout << "loading file: " << filename << std::endl;
  if ( !input ) 
  {
    std::cerr << "No input" << std::endl;
    exit ( 1 ) ;
  }
  else if ( !(input >> poly ) )
  {
    std::cerr << "couldn't load" << std::endl;
    exit ( 1 ) ;
  }
  else if ( poly.empty() )
  {
    std::cerr << "poly is empty" << std::endl;
    exit ( 1 ) ;
  }
  std::cout << "Loaded file. Vertices: " << poly.size_of_vertices() 
            << " Facets: " << poly.size_of_facets() 
            << " Half Edges: "  << poly.size_of_halfedges()
            << " Border Edges: " << poly.size_of_border_edges() 
            << std::endl;

  return poly;
}


Polyhedron createPolyhedron ( Reconstruction recon )
{

 Polyhedron poly;

 std::vector< Point > pointvector;
  for ( Point_iterator pit = recon.points_begin(); pit != recon.points_end(); ++pit )
  {
    Point q ( pit[0][0], pit[0][1], pit[0][2]);
    pointvector.push_back ( q );
  }

  for ( Facet_iterator it = recon.facets_begin( ); it != recon.facets_end(  ); ++it )
  {
    size_t i1 = it[0][0];
    size_t i2 = it[0][1];
    size_t i3 = it[0][2];
    poly.make_triangle ( pointvector[i1], pointvector[i2], pointvector[i3] );
  }

  return poly;
}


Polyhedron createPolyhedron (std::vector<Point_3> points, std::vector<Facet> facets )
{

  Polyhedron poly;
  std::vector< Point > pointvector;
  for ( size_t pit = 0; pit < points.size(); pit++ )
  {
    Point q ( points[pit][0], points[pit][1], points[pit][2] );
    pointvector.push_back ( q );
  }

  for ( size_t fit = 0; fit < facets.size(); fit++ )
  {
    size_t i1 = facets[fit][0];
    size_t i2 = facets[fit][1];
    size_t i3 = facets[fit][2];
    poly.make_triangle ( pointvector[i1], pointvector[i2], pointvector[i3] );
  }

  return poly;
}

/////////////////// Mesh Generation //////////////////
Polyhedron advancingFrontSurfaceReconstruction( std::vector<Point_3> points, Perimeter perimeter)
{
  std::vector<Facet> facets ;
  CGAL::advancing_front_surface_reconstruction(points.begin(),
                                               points.end(),
                                               std::back_inserter(facets),
                                               perimeter);
   Polyhedron poly = createPolyhedron ( points, facets );

  return poly;

}

Polyhedron scaleSpaceReconstruction ( std::vector<Point> points, int scale=4 )
{

  CGAL::Timer t;
  t.start();

  // Construct the mesh in a scale space.
  Reconstruction reconstruct (points.begin(), points.end());
  reconstruct.increase_scale(scale);
  reconstruct.reconstruct_surface();

  Polyhedron poly = createPolyhedron ( reconstruct );
  
  return poly;

}



///////////// Mesh Refinement /////////////////////
void fillholes ( Polyhedron &poly , float density )
{

  unsigned int nb_holes = 0;

  std::cout << "Poly size before hole fill: " << poly.size_of_facets() << " " << poly.size_of_vertices() << std::endl;
  BOOST_FOREACH(Halfedge_handle h, halfedges(poly))
  {
    if(h->is_border())
    {
      std::vector<Facet_handle>  patch_facets;
      std::vector<Vertex_handle> patch_vertices;
      
      bool success = CGAL::cpp11::get<0>(
        CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                  poly,
                  h,
                  std::back_inserter(patch_facets),
                  std::back_inserter(patch_vertices),
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, poly)).
                  density_control_factor ( density ) ) 
                                        );
                  
      ++nb_holes;
    }
  }

  std::cout << nb_holes << " holes have been filled. " << std::endl;
}

void refinemesh ( Polyhedron &poly, float density )
{
  std::vector<Polyhedron::Facet_handle>  new_facets;
  std::vector<Vertex_handle> new_vertices;
  CGAL::Polygon_mesh_processing::refine(poly, faces(poly), 
                  std::back_inserter(new_facets),
                  std::back_inserter(new_vertices),
                  CGAL::Polygon_mesh_processing::parameters::density_control_factor(density));

  std::cout << "Refinement added " << new_vertices.size() << " vertices." << std::endl;
  // return poly;
}

void extract_k_ring( Vertex_handle v, int k, std::vector<Vertex_handle>& qv )
{
  std::map<Vertex_handle, int>  D;
  qv.push_back(v);
  D[v] = 0;
  std::size_t current_index = 0;
  int dist_v;
  while (current_index < qv.size() && (dist_v = D[qv[current_index]]) < k)
  {
    v = qv[current_index++];
    Polyhedron::Halfedge_around_vertex_circulator e(v->vertex_begin()), e_end(e);
    do {
      Vertex_handle new_v = e->opposite()->vertex();
      if (D.insert(std::make_pair(new_v, dist_v + 1)).second) {
        qv.push_back(new_v);
      }
    } while (++e != e_end);
  }
}

void fair ( Polyhedron &poly )
{
  std::cout << "Begin Fairing" << std::endl;

  std::cout << "Building vertex object" << std::endl;
  Polyhedron::Vertex_iterator v = poly.vertices_begin();

  std::advance(v, 82/*e.g.*/);
  std::vector<Vertex_handle> region;

  std::cout << "Extracting k-ring" << std::endl;
  extract_k_ring(v, 12/*e.g.*/, region);
  
  bool success = CGAL::Polygon_mesh_processing::fair(poly, region);
  std::cout << "Fairing : " << (success ? "succeeded" : "failed") << std::endl;

}

C3t3 highLevelMesh ( 
                      Polyhedron poly, 
                      std::vector<float> mesh_criteria, 
                      std::vector<bool> mesh_parameters, 
                      std::vector<float> time_limits,
                      std::stringstream &title)
{

  Mesh_domain domain(poly);
  
  std::cout << "Generating mesh criteria ....";
  Mesh_criteria criteria (  
                            facet_angle=mesh_criteria[0], 
                            facet_size=mesh_criteria[1], 
                            facet_distance=mesh_criteria[2], 
                            cell_radius_edge_ratio=mesh_criteria[3]
                          );
  bool empty_criteria = true;
  for ( int i = 0; i < mesh_criteria.size()-1; i++ )
  {
    if ( mesh_criteria[i] != 0 )
      empty_criteria = false;
      break;
  }
  if ( empty_criteria )
  {
    std::cout << "Empty mesh criteria......" << std::endl;
    Mesh_criteria criteria ();
  }
  else
  {
    title << "facet_angle"<<mesh_criteria[0]<<"facet_size"<<mesh_criteria[1]<<"facet_distance"<<mesh_criteria[2]
        <<"cell_radius_edge_ratio"<<mesh_criteria[3];   
  }


  std::cout << "..........done"<<std::endl;



  C3t3 c3t3;
  if ( mesh_parameters[0] )
  {
    std::cout << "Mesh gen & lloyd optimization....." <<std::endl;
    c3t3 = CGAL::make_mesh_3<C3t3>(
                                    domain, criteria,

                                    lloyd(time_limit=time_limits[0], 
                                          max_iteration_number = mesh_criteria[4],
                                          convergence = 0.001
                                          ),

                                    no_perturb(),
                                    no_exude ()
                                  );
    title << "lloyd" << time_limits[0]<<"max_iter"<<mesh_criteria[4];

    std::cout<< "........done."<<std::endl;
  }
  else
  {
    std::cout << "Generating mesh........." << std::endl;
    c3t3 = CGAL::make_mesh_3<C3t3> ( domain, criteria, time_limit = 10 );
    std::cout << "               .............done" << std::endl;
  }

  if ( mesh_parameters[1] )
  {
    std::cout << "Delaunay Optimization....." <<std::endl;
    CGAL::odt_optimize_mesh_3<C3t3> ( c3t3, domain, time_limit = time_limits[1] );
    title << "odt" << time_limits[1];
    std::cout<< "........done."<<std::endl;
  }

  if ( mesh_parameters[2] )
  {
    std::cout << "Perturbing....." <<std::endl;
    CGAL::perturb_mesh_3<C3t3> ( c3t3, domain, time_limit = time_limits[2] );
    title << "perturb" << time_limits[2];
    std::cout<< "........done."<<std::endl;
  }

  if ( mesh_parameters[3] == true )
  {
    std::cout << "Exuding............"<<std::endl;
    CGAL::exude_mesh_3<C3t3> ( c3t3, time_limit = time_limits[3] );
    std::cout << "..............done" << std::endl;
    title<< "exude" << time_limits[3];
  }


  return c3t3;

}