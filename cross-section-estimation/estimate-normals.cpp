#include "includes.h"

void newtonRaphson ( float &cost, Polygon &polygon, pointCloud &pointset , float tolerance=0.01)
{
  pointCloud temp = pointset;
  float del = 10000;
  float prevdel = 0;
  float oldcost = 10000;
  while ( (del > tolerance) && (abs(del - prevdel) > 0.5) ) 
  {
    
    costfunction ( cost, polygon, pointset, temp);
    std::cout << "Move cost: " << cost << std::endl;

    prevdel = del;
    del = polygon.rewrite ( cost, oldcost );
    float oldcost = cost;
    std::cout << del << " " << prevdel << " "  << tolerance << std::endl;
  }
  pointset = temp;
}

int main ( int argc, char** argv)
{
  std::stringstream readfile ( argv[1] );
  std::vector<std::string> parameters = readparameters ( readfile.str() );

  std::string fullfile    	=   parameters[0];
  // float radius 			  	=	stof ( parameters[1] );
  float tolerance			=	stof ( parameters[1] );
  float del_tolerance = stof ( parameters[2] );
  int 	minclust 			= 	1;
  pointCloud::Ptr cloud 	= 	loadcloud ( fullfile );
  pointCloud::Ptr newcloud ( new pointCloud ), tempcloud ( new pointCloud );
  newcloud = cloud;
  ////////////// Build Initial Beam Cross-Section //////////////////////
  Polygon poly (5, 3, 2, 1, Eigen::Vector3f (0,0,0) );

  float cost = 0;
  ////////// Determining Cost of fitting points to beam ///////////
  /*    Our goal here to is to find the optimal beam cross-section parameters that minimize 
        the amount of distance traversed by the points within range of the cross-section plane
  */ 
  // visualize ( cloud );
  std::cout << "Moving points to polygon and calculating cost " << std::endl;
    pointsnearplane ( poly, *cloud, *tempcloud, tolerance);
    std::cout << "cloud size: " << cloud->points.size() << " temp cloud size: " << tempcloud->points.size() <<std::endl;
    newtonRaphson ( cost, poly, *tempcloud, del_tolerance );
    *newcloud += *tempcloud;


  visualize ( tempcloud );


  return 0;
}
