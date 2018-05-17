#include "includes.h"

void newtonRaphson ( float &cost, Polygon &polygon, pointCloud &pointset , float costtolerance = 0, float difftolerance=0.01)
{
  pointCloud temp = pointset;
  bool crit1 = true;
  bool crit2 = true;
  float oldcost;
  int count =0;
  costfunction ( cost, polygon, pointset, temp);
  polygon.set( 10, 5, 2, 1, Eigen::Vector3f(0,0,0) );
  while ( crit1 && crit2 && ( count < 5 ) )
  {
    oldcost = cost;
    costfunction ( cost, polygon, pointset, temp);
    std::cout << "Move cost: " << cost << " previous: " << oldcost << std::endl;

    polygon.rewrite ( cost, oldcost );


    crit1 = cost > costtolerance;
    crit2 = abs(cost - oldcost) > difftolerance;
    std::cout << "cost diff: " << abs(cost - oldcost) << " " << difftolerance << std::endl;
    count++;
  }
  // costfunction ( cost, polygon, pointset, temp);
  pointset = temp;
  std::cout << "Final values:\n"
    << "thickness: "<< polygon.thickness  << "\n"
    << "width: "    << polygon.width      << "\n"
    << "height 1: " << polygon.height1    << "\n"
    << "height 2: " << polygon.height2    << "\n"
    << "theta: "    << polygon.theta      << "\n"
    << "phi: "      << polygon.phi        << "\n"
    << std::endl;
    polygon.makeCloud();

}

int main ( int argc, char** argv)
{
  std::stringstream readfile ( argv[1] );
  std::vector<std::string> parameters = readparameters ( readfile.str() );

  std::string fullfile    	=   parameters[0];
  // float radius 			  	=	stof ( parameters[1] );
  float tolerance			=	stof ( parameters[1] );
  float cost_tol = stof ( parameters[2] );
  float dif_tol = stof ( parameters[3] );
  int 	minclust 			= 	1;
  pointCloud::Ptr cloud 	= 	loadcloud ( fullfile );
  pointCloud::Ptr newcloud ( new pointCloud ), tempcloud ( new pointCloud );

  ////////////// Build Initial Beam Cross-Section //////////////////////
  Polygon poly (5, 3, 2, 1, Eigen::Vector3f (0,0,0) );
  
  float cost = 0;
  ////////// Determining Cost of fitting points to beam ///////////
  /*    Our goal here to is to find the optimal beam cross-section parameters that minimize 
        the amount of distance traversed by the points within range of the cross-section plane
  */ 
  visualize ( cloud );
    pointsnearplane ( poly, *cloud, *tempcloud, tolerance);
    std::cout << "cloud size: " << cloud->points.size() << " temp cloud size: " << tempcloud->points.size() <<std::endl;
    newtonRaphson ( cost, poly, *tempcloud, cost_tol, dif_tol );
    *newcloud += *tempcloud;
  pointCloud::Ptr vertex ( new pointCloud );
  *vertex += poly.vertices;
  std::vector<pointCloud::Ptr> set;
  set.push_back(vertex);
  visualize ( newcloud );
  visualize ( vertex );

  writetoXYZ ( "lcloud", set);

  return 0;
}
