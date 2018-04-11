#include "includes.h"

int main(int argc, char** argv)
{
  const char* filename = argv[1];
  const char* o  = argv[2];
  std::stringstream outfile ;
  outfile << o ;

  // Read .off file to polyhedron
  Polyhedron poly = offToPoly ( filename );

  // Incrementally fill the holes
  poly = fillholes( poly ) ;
  std::cout << outfile.str();
  std::ofstream out(outfile.str());
  out.precision(17);
  out << poly << std::endl;
  return 0;
}