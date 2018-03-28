
/*
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/IO/read_xyz_points.h>

#include <CGAL/property_map.h>
#include <vector>
#include <fstream>

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Surface_mesh_default_criteria_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/IO/Complex_2_in_triangulation_3_file_writer.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Gray_level_image_3.h>
#include <CGAL/Implicit_surface_3.h>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// default triangulation for Surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;
// c2t3
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;
typedef Tr::Geom_traits GT;
typedef CGAL::Gray_level_image_3<GT::FT, GT::Point_3> Gray_level_image;
typedef CGAL::Implicit_surface_3<GT, Gray_level_image> Surface_3;


int main(int argc, char*argv[])
{
  
  // Reads a .xyz point set file in points[].
  std::vector<Point> points;
  std::vector<Vector> normals;
  const char* fname = (argc>1)?argv[1]:"data/fin90_with_PCA_normals.xyz";
  std::ifstream stream(fname);
  Point p;
  Vector v;
  while(stream >> p >> v){
    points.push_back(p);
    normals.push_back(v);
  } 
  std::cout << points.size() << " input points" << std::endl;
  std::vector<std::size_t> indices(points.size());
  for(std::size_t i = 0; i < points.size(); ++i){
    indices[i] = i;
  }
  
  return EXIT_SUCCESS;
  
}
*/


#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
// default triangulation for Surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;
// c2t3
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;
typedef Tr::Geom_traits GT;
typedef GT::Sphere_3 Sphere_3;
typedef GT::Point_3 Point_3;
typedef GT::FT FT;
typedef FT (*Function)(Point_3);
// typedef CGAL::Implicit_surface_3<GT, Function> Surface_3;
FT sphere_function (Point_3 p) {
  const FT x2=p.x()*p.x(), y2=p.y()*p.y(), z2=p.z()*p.z();
  return x2+y2+z2-1;
}
int main() {
  Tr tr;            // 3D-Delaunay triangulation
  C2t3 c2t3 (tr);   // 2D-complex in 3D-Delaunay triangulation
  // defining the surface
  // Surface_3 surface(sphere_function,             // pointer to function
  //                   Sphere_3(CGAL::ORIGIN, 2.)); // bounding sphere
  // Note that "2." above is the *squared* radius of the bounding sphere!
  // defining meshing criteria
  // CGAL::Surface_mesh_default_criteria_3<Tr> criteria(30.,  // angular bound
  //                                                    0.1,  // radius bound
  //                                                    0.1); // distance bound
  // // meshing surface
  // CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Non_manifold_tag());
  // std::cout << "Final number of points: " << tr.number_of_vertices() << "\n";
}
