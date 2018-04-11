#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>


#include <CGAL/IO/read_off_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/tuple.h>
#include <CGAL/Polyhedron_3.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <dlib/matrix.h>


namespace fs = ::boost::filesystem;
using namespace std;
using namespace dlib;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;

typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> colorCloud;



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

void mkdir ( std::string directory )
{
  fs::path dir (directory);
  if ( !fs::create_directory(dir))
  {
    std::cerr << "Couldn't make folder:\n" << directory << "\n";
  }
  else
  {
    std::cout << "created folder: " << directory << "\n";
  }
}

///////////// CGAL ///////////////////////
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
  std::cout << "loaded pointset of size: " << pointset.size() << std::endl;
  return pointset;
}

Polyhedron offToPoly ( const char* filename )
{
  std::ifstream input(filename);
  Polyhedron poly;
  if ( !input || !(input >> poly) || poly.empty() ) {
    std::cerr << "Not a valid off file." << std::endl;
  }
  return poly;
}

/////////////////// PCL ////////////////
pointCloud::Ptr loadXYZ ( std::string filename )
{
  pointCloud::Ptr cloud ( new pointCloud );
  std::ifstream inFile;
  char x;
  
  inFile.open ( filename );

  if (!inFile) {
      std::cerr << "Unable to open xyz file";
      exit(1);
  }

  std::vector<std::string> values ;
  std::vector<pcl::PointXYZ> pointset ;
  std::string i;

  while (inFile >> x) 
  {
      // std:cout << x ;
      if ( x == ' ' || x == ',' || x == '\t' )
        inFile  >> x ;

      i += x ;
      if ( inFile.peek () == ' ' || inFile.peek () == ',' )
      {
        // inFile >> x;
        values.push_back(i);
        i = "";
      }

      if ( inFile.peek () == '\n' )
      {
        // std::cout << values[0] << " " << values[1] << " " << values[2] << std::endl;
        values.push_back(i);
        pcl::PointXYZ point;
        point.x = stof(values[0]);
        point.y = stof(values[1]);
        point.z = stof(values[2]);

        pointset.push_back ( point );


        values.clear () ;
        i = "";
      }
  }
  cloud->width = pointset.size ();
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize (cloud->width * cloud->height);

  for ( size_t i = 0; i < pointset.size (); i ++ )
  {
    cloud->points[i] = pointset[i];
  }

  std::cout << "Cloud Size: " << cloud->points.size() << std::endl;
  inFile.close ( );
  return cloud;

}

pointCloud::Ptr loadcloud(std::string loadfile)
{
    std::string filename = loadfile;
    pointCloud::Ptr newcloud ( new pointCloud );
    
    std::string ex = fs::extension ( filename ) ;


    if ( ex == ".xyz" )
    {
       newcloud = loadXYZ ( filename ) ;
    }
    else
    {
      pcl::PCDReader reader;
      reader.read (filename, *newcloud);
    }

    return newcloud;
}

void writetoPCD(std::stringstream ss, pointCloud::Ptr cloud)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (ss.str (), *cloud, false);
}

void writetoPCD (std::string directory, std::vector<colorCloud::Ptr> cloudvector)
{


  mkdir ( directory );
  pcl::PCDWriter writer;
  for (int i = 0; i < cloudvector.size(); i++)
  {
    std::stringstream title ;
    title << directory << i << ".pcd" ;

    writer.write<pcl::PointXYZRGB> (title.str (), *cloudvector[i], false);
  }
  std::cout << "Wrote " << cloudvector.size() << " to folder " << directory << "\n";
}

void writetoXYZ ( std::string directory, std::vector<colorCloud::Ptr> cloudvector )
{
  mkdir( directory );
  for (int i = 0; i < cloudvector.size(); i++)
  {
    std::stringstream title ;
    std::ofstream file ;
    title << directory << i << ".xyz" ;
    file.open ( title.str () );
    for ( size_t j = 0; j < cloudvector[i]->points.size(); j++ )
    {
      
      file << cloudvector[i]->points[j].x << " " << cloudvector[i]->points[j].y << " "
      << cloudvector[i]->points[j].z << "\n";
    }

    file.close ();
  }

  std::cout << "Wrote " << cloudvector.size() << " to folder " << directory << "\n";

}

void writetoPCD(std::stringstream ss, colorCloud::Ptr cloud)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (ss.str (), *cloud, false);
}

/////////////////// dlib ////////////////
typedef matrix<double,3,1> pointMatrix;

std::vector<pointMatrix> pclToMatrix ( pointCloud::Ptr cloud )
{
    std::vector<pointMatrix> samples;
    pointMatrix m;

   for ( size_t i = 0; i < cloud->points.size(); i++ )
   {
       m(0) = cloud->points[i].x;
       m(1) = cloud->points[i].y;
       m(2) = cloud->points[i].z;
       samples.push_back ( m );
   }
   return samples;
}