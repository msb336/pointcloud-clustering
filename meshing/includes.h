#include <iostream>
#include <fstream>
#include <algorithm>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/tuple.h>
#include <boost/lexical_cast.hpp>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Timer.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <vector>
#include <boost/foreach.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;


typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3  Point_3;
typedef CGAL::cpp11::array<std::size_t,3> Facet;
typedef K::Vector_3 Vector;
typedef std::pair<Point_3, Vector> Pwn;

typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>    Reconstruction;
typedef Kernel::Point_3 Point;
typedef Reconstruction::Facet_const_iterator                   Facet_iterator;


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


void advancingFrontSurfaceReconstruction( std::vector<Point_3> points, Perimeter perimeter, std::string savefile )
{
  std::vector<Facet> facets ;
  CGAL::advancing_front_surface_reconstruction(points.begin(),
                                               points.end(),
                                               std::back_inserter(facets),
                                               perimeter);

  std::ofstream out ( savefile );
  out << "OFF\n";
  out << points.size() << " " << facets.size() << " " << 0 << "\n";
  std::copy(points.begin(),
            points.end(),
            std::ostream_iterator<Point_3>(out, "\n"));

  std::copy(facets.begin(),
            facets.end(),
            std::ostream_iterator<Facet>(out, "\n"));
}

Reconstruction scaleSpaceReconstruction ( std::vector<Point> points , std::string savefile)
{
  CGAL::Timer t;
  t.start();

  // Construct the mesh in a scale space.
  Reconstruction reconstruct (points.begin(), points.end());
  reconstruct.increase_scale(4);
  reconstruct.reconstruct_surface();
  std::cerr << "done in " << t.time() << " sec." << std::endl;

  t.reset();

  std::ofstream out ( savefile );
  out << reconstruct;
  std::cerr << "Writing result in " << t.time() << " sec." << std::endl;
  std::cerr << "Done." << std::endl;
  return reconstruct;

}

std::vector<Point> loadexact ( const char* fname )
{
  std::ifstream in ( fname ) ;
  std::vector<Point> pointset;

   std::copy(std::istream_iterator<Point>(in),
            std::istream_iterator<Point>(),
            std::back_inserter(pointset));
  std::cout << "loaded pointset of size: " << pointset.size() << std::endl;
  return pointset;
}

std::vector<Point_3> loadsimple ( const char* fname )
{
  std::ifstream in ( fname ) ;
  std::vector<Point_3> pointset;

   std::copy(std::istream_iterator<Point_3>(in),
            std::istream_iterator<Point_3>(),
            std::back_inserter(pointset));
  std::cout << "loaded pointset of size: " << pointset.size() << std::endl;
  return pointset;
}

Polyhedron offToPoly ( const char* filename )
{
  std::ifstream input(filename);
  Polyhedron poly;
  // if ( !input || !(input >> poly) || poly.empty() ) {
  //   std::cerr << "Not a valid off file." << std::endl;
  // }
  return poly;
}

Polyhedron fillholes ( Polyhedron poly )
{
  unsigned int nb_holes = 0;
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
                  geom_traits(Kernel())) );
      std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
      std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
      std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
      ++nb_holes;
    }
  }
  std::cout << std::endl;
  std::cout << nb_holes << " holes have been filled" << std::endl;
  return poly ;
}