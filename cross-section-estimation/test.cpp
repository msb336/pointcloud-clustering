#include "potentiallynecessary.h"
#include <pcl/common/transforms.h>
class BeamCost
{
  public:
    std::vector<float> cost;
    std::vector<float> oldcost;
    Polygon poly;
    NormalCloud::Ptr ncloud;
    BeamCost ( Polygon, NormalCloud::Ptr, std::vector<float>);
};
  BeamCost::BeamCost ( Polygon beamobject, NormalCloud::Ptr plane_cloud_with_normals, std::vector<float> initcost )
  { poly = beamobject; ncloud = plane_cloud_with_normals;
    cost = initcost;
    for(int i =0;i<2;i++){oldcost.push_back(500);} }



  bool newtonsimple ( BeamCost &beam, float multiplier, std::string modifier, float del_tolerance=0.1, float dcost_tol = 0.001 )
{


  float dcost1 (beam.cost[0] - beam.oldcost[0]);
  float dcost2 (beam.cost[1] - beam.oldcost[1]);
  beam.oldcost = beam.cost;
  if (fabs(dcost1) < dcost_tol || fabs(dcost2) < dcost_tol)
  {std::cout << dcost1 << " " << dcost2 << std::endl;return true;}
  else
  {
    float del;

    /*
    float dw = beam.poly.width - beam.poly.oldwidth;
    beam.poly.oldwidth = beam.poly.width;
    beam.poly.width = beam.poly.width - multiplier*beam.cost[0]*dw/dcost1;
    */

    float dh = beam.poly.height - beam.poly.oldheight;
    beam.poly.oldheight = beam.poly.height;
    beam.poly.height = beam.poly.height - multiplier*beam.cost[1]*dh/dcost2;
    beam.poly.width = beam.poly.height;
    
    float dt = beam.poly.thickness - beam.poly.oldthickness;
    beam.poly.oldthickness = beam.poly.thickness;
    float costsum = beam.cost[0]+beam.cost[1];
    float dsum    = dcost1+dcost2;
    beam.poly.thickness = beam.poly.thickness - multiplier*(costsum)*dt/(dsum);

    // beam.poly.maxtest();

    del = sqrt ( dt*dt + dh*dh );// + dw*dw );
  	beam.poly.calculatePositions();
    
    /*
    std::cout << "Current Cost: " << beam.cost[0] << " " << beam.cost[1] << " " 
    << " Old cost: " << beam.oldcost[0] << " " <<beam.oldcost[1] << std::endl;
    std::cout << "value change: " << del << " dcost1: " << dcost1 << " dcost2: " << dcost2 << std::endl;
    std::cout << fabs(del) << " < " << del_tolerance << " = " << ( fabs(del) < del_tolerance ) << std::endl;
  	*/

    return ( fabs(del) < del_tolerance );
  }
	
}


void modcost ( BeamCost &beam, PointCloud &projected, float dot_tolerance)
{

  beam.cost[0] = 0; beam.cost[1] = 0;
	for (int i = 0; i < beam.ncloud->size(); i++)
  {
       projectToPolygon ( beam.cost, beam.poly, beam.ncloud->points[i], projected.points[i], dot_tolerance);
  }
  beam.cost[0] = 50*beam.cost[0] / beam.ncloud->size(); beam.cost[1] = 50*beam.cost[1] / beam.ncloud->size(); 
}


















int main ( )
{

  ///////////// Read in Parameters
  std::vector<std::string> parameters = readparameters ( "../parameters.txt" );

  std::string fullfile    	  =         parameters[0]  ;
  float planar_tolerance			=	 stof ( parameters[1] );
  float cost_tol              =  stof ( parameters[2] );
  float dif_tol               =  stof ( parameters[3] );
  float multi                 =  stof ( parameters[4] );
  int max_iter                =  stoi ( parameters[5] );
  float initial_width         =  stof ( parameters[6] );
  float initial_height        =  stof ( parameters[7] );
  float initial_thickness     =  stof ( parameters[8] );
  float step_size             =  stof ( parameters[9] );
  float dot_tolerance         =  stof ( parameters[10]);

  NormalCloud::Ptr full_cloud ( new NormalCloud);
  load_set ( fullfile, *full_cloud );
  downsample(full_cloud);
  PointCloud::Ptr just_the_points ( new PointCloud );
  Normals::Ptr just_the_normals ( new Normals );
  pcl::copyPointCloud ( *full_cloud, *just_the_points);
  pcl::copyPointCloud ( *full_cloud, *just_the_normals);
  visualize(just_the_points, just_the_normals);

  //////// Find Largest plane in beam ///////////
  pcl::ModelCoefficients::Ptr largest_cloud_plane = getLargestPlane ( full_cloud, planar_tolerance );

  ///////////////// Calculate Rotation and rotate cloud to origin /////////////
  Eigen::Transform<float, 3, Eigen::Affine> transformation_to_origin = rotateToOrigin ( largest_cloud_plane );

  //////// Transform cloud to origin ///////
  NormalCloud::Ptr rotated_cloud_with_normals ( new NormalCloud ); // Change this to NormalCloud
  // pcl::transformPointCloud ( *full_cloud, *rotated_cloud_with_normals, transformation_to_origin );
  *rotated_cloud_with_normals = *full_cloud;

  ///////// Find Point with Largest y distance and calculate translation to origin //////
  moveToOrigin ( *rotated_cloud_with_normals, *rotated_cloud_with_normals );
  PointCloud::Ptr points_after_shift ( new PointCloud );
  pcl::copyPointCloud ( *rotated_cloud_with_normals, *points_after_shift);


  


  /////////////// Build beam cost object /////////////
  
  Polygon latestdimensions ( initial_width,initial_height, initial_thickness, Eigen::Vector3f(0,0,0) );

  PointT maxpt;
  PointT minpt;
  getExtremes3D ( *rotated_cloud_with_normals, minpt, maxpt );
  float L = maxpt.x;
  // std::cout << "Max point: " << L << std::endl;
  std::vector<PointCloud::Ptr> clouds;

  PointCloud::Ptr result_cloud ( new PointCloud );
  float h=0; float t=0; int N=0;
  for (float i = 0; i<L; i+=step_size)
  {
    std::cout << "step " << i << " of " << maxpt.x << std::endl;
    //////////// Create beam object ///////////
  	Polygon poly ( latestdimensions.height, latestdimensions.width, latestdimensions.thickness, Eigen::Vector3f (i,0,0) );
    poly.makeCloud();

    ////////////// Determine Plane coefficinets for polygon //////////////////
   	pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients );
   	pointsToPlane( poly.lines[0].begin, poly.lines[1].begin, poly.lines[2].begin, plane );

    ///////// Find Points within tolerance of plane ///////////////////
    NormalCloud::Ptr plane_cloud_with_normals ( new NormalCloud );
    pointsnearplane ( poly, *rotated_cloud_with_normals, *plane_cloud_with_normals, planar_tolerance );
    // std::cout << "plane size: " << plane_cloud_with_normals->size() << " " << rotated_cloud_with_normals->size() << std::endl;

    
    /////////////// Build beam cost object /////////////
    std::vector<float> initial_cost;
    for(int i=0;i<2;i++){initial_cost.push_back(5);}
    BeamCost lbeam ( poly, plane_cloud_with_normals, initial_cost);
    




    PointCloud::Ptr projectedcloud ( new PointCloud) , testcloud ( new PointCloud );
    pcl::copyPointCloud ( *plane_cloud_with_normals, *projectedcloud );
    pcl::copyPointCloud ( *projectedcloud, *testcloud );


    bool dif = true;
    bool max = true;
    int iter = 0;

    while ( dif && max && iter < max_iter)
    {
    	modcost ( lbeam, *projectedcloud, dot_tolerance );
    	lbeam.poly.makeCloud();
    	clouds.clear();
      dif = !newtonsimple ( lbeam, multi, "", dif_tol );

    	max = sqrt ( lbeam.cost[0]*lbeam.cost[0] + lbeam.cost[1]*lbeam.cost[1]) > cost_tol;

      iter++;
    }
    
    std::cout << "\n\niterators terminated."  << " dif: "             << dif     << " max: "                << max     << " iter: " << iter 
              << "\ncost: "               << lbeam.cost[0]        << " "     << lbeam.cost[1]           << "\n" << std::endl;
              /*
              <<" width: "                << lbeam.poly.width     << " "     << lbeam.poly.oldwidth     << "\n"
              << " height: "              << lbeam.poly.height    << " "     << lbeam.poly.oldheight    << "\n"
              << " thickness: "           << lbeam.poly.thickness << " "     << lbeam.poly.oldthickness <<"\n\n\n" << std::endl;
              */
    h+=lbeam.poly.height;t+=lbeam.poly.thickness;
    N++;
    std::cout << "current height: " << lbeam.poly.height
              << " current thickness: " << lbeam.poly.thickness 
              << "\naverage height: " << h/N 
              << " average thickness: " << t/N << std::endl;

    // visualize(testcloud);

    clouds.clear();
    // clouds.push_back ( lbeam.poly.vertices );
    *result_cloud+=*lbeam.poly.vertices;

    clouds.push_back(testcloud);
    clouds.push_back (projectedcloud);
    visualize(clouds);
    
    latestdimensions = lbeam.poly;
  }
  // std::cout << "average height: " << h/N << " average thickness: " << t/N << std::endl;
  visualize ( result_cloud, true );
  return 0;
}