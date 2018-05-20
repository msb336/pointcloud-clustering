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
    for(int i =0;i<2;i++){oldcost.push_back(0);} }


// bool newtonsimple ( std::vector<float> cost, std::vector<float> &oldcost, Polygon &p, float multiplier, float tolerance = 0.1 )
// bool newtonsimple ( float cost, float &oldcost, Polygon &p, float tolerance = 0.1 )
  bool newtonsimple ( BeamCost &beam, float multiplier, std::string modifier, float tolerance=0.1 )
{


  float dcost1 (beam.cost[0] - beam.oldcost[0]);
  float dcost2 (beam.cost[1] - beam.oldcost[1]);
  float del;
  if ( modifier == "width" )
  {
    float dw = beam.poly.width - beam.poly.oldwidth;
    beam.poly.oldwidth = beam.poly.width;
    beam.poly.width = beam.poly.width - multiplier*beam.cost[0]*dw/dcost1;
    del = dw;
    std::cout << "width mod ";// << std::endl;
  }
  else if ( modifier == "height" )
  {
    float dh = beam.poly.height - beam.poly.oldheight;
    beam.poly.oldheight = beam.poly.height;
    beam.poly.height = beam.poly.height - multiplier*beam.cost[1]*dh/dcost2;
    del = dh;
    std::cout << "height mod ";// << std::endl;
  }
  else if ( modifier == "thickness" )
  {
    float dt = beam.poly.thickness - beam.poly.oldthickness;
    beam.poly.oldthickness = beam.poly.thickness;
    beam.poly.thickness = beam.poly.thickness - multiplier*(beam.cost[1]+beam.cost[2])*dt/(dcost1 + dcost2);
    del = dt;
    std::cout << "thickness mod: " << multiplier*(beam.cost[1]+beam.cost[2])*dt/(dcost1 + dcost2) << " " ; 
  }
  else
  { del = 0; }

  // beam.poly.maxtest();
	beam.poly.calculatePositions();
  std::cout << "x value change: " << del << " cost 1: " << dcost1 << " cost 2: " << dcost2 << std::endl;

	return ( del < tolerance ); 
	
}


void modcost ( BeamCost &beam, pointCloud &projected)
{

  beam.cost[0] = 0; beam.cost[1] = 0;
	for (int i = 0; i < beam.ncloud->size(); i++)
  {
       projectToPolygon ( beam.cost, beam.poly, beam.ncloud->points[i], projected.points[i]);
  }
}
int main ( )
{

  ///////////// Read in Parameters
  std::vector<std::string> parameters = readparameters ( "../parameters.txt" );

  std::string fullfile    	=   parameters[0];
  // float radius 			  	=	stof ( parameters[1] );
  float planar_tolerance			=	 stof ( parameters[1] );
  float cost_tol              =   stof ( parameters[2] );
  float dif_tol               =   stof ( parameters[3] );
  float multi                 =   stof ( parameters[4] );
  float radius                =   stof ( parameters[5] );
  int max_iter                    =   stoi ( parameters[6] );
  int 	minclust 			        = 	1;
  pointCloud::Ptr full_cloud 	    = 	loadcloud ( fullfile );
  // visualize ( full_cloud );
  //////// Find Largest plane in beam ///////////
  pcl::ModelCoefficients::Ptr largest_cloud_plane = getLargestPlane ( full_cloud, planar_tolerance );

  ///////////////// Calculate Rotation and rotate cloud to origin /////////////
  Eigen::Transform<float, 3, Eigen::Affine> transformation_to_origin = rotateToOrigin ( largest_cloud_plane );

  //////// Transform cloud to origin ///////
  pointCloud::Ptr rotated_cloud ( new pointCloud );
  pcl::transformPointCloud ( *full_cloud, *rotated_cloud, transformation_to_origin );

  ///////// Find Point with Largest y distance and calculate translation to origin //////
  moveToOrigin ( *rotated_cloud, *rotated_cloud );

  //////////////// Calculate Normals of the cloud///////////
  NormalCloud::Ptr rotated_cloud_with_normals ( new NormalCloud );
  Normals::Ptr just_the_normals ( new Normals );
  computenormals(rotated_cloud, *just_the_normals, *rotated_cloud_with_normals, radius);
  std::cout << "number of normals: " << just_the_normals->size() << std::endl;
  visualize ( rotated_cloud, just_the_normals );
  

  //////////// Create beam object ///////////
	 Polygon poly (5, 5, 0.5, Eigen::Vector3f (0,0,0) );
   poly.makeCloud();


  ////////////// Determine Plane coefficinets for polygon //////////////////
 	pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients );
 	pointsToPlane( poly.lines[0].begin, poly.lines[1].begin, poly.lines[2].begin, plane );

  ///////// Find Points within tolerance of plane ///////////////////
  NormalCloud::Ptr plane_cloud_with_normals ( new NormalCloud );
  pointsnearplane ( poly, *rotated_cloud_with_normals, *plane_cloud_with_normals );
  std::cout << plane_cloud_with_normals->size() << " " << full_cloud->size() << std::endl;

  
  /////////////// Build beam cost object /////////////
  std::vector<float> initial_cost;
  for(int i=0;i<2;i++){initial_cost.push_back(5);}
  BeamCost lbeam ( poly, plane_cloud_with_normals, initial_cost);
  
  pointCloud::Ptr projectedcloud ( new pointCloud) , testcloud ( new pointCloud );
  pcl::copyPointCloud ( *plane_cloud_with_normals, *projectedcloud );
  pcl::copyPointCloud ( *projectedcloud, *testcloud );

  ////////////// Initialize cost function ////////////////////


  std::vector<pointCloud::Ptr> clouds;
  std::vector<std::string> mod_variable;
  mod_variable.push_back ( "thickness" );mod_variable.push_back ( "width" );mod_variable.push_back ( "height" );

  
  for ( int mod_index = 0; mod_index < 3; mod_index++ )
  {
    bool dif = true;
    bool max = true;
    int iter = 0;
    while ( dif && max && iter < max_iter)
    {
    	modcost ( lbeam, *projectedcloud );
    	lbeam.poly.makeCloud();
    	clouds.clear();
      clouds.push_back ( lbeam.poly.vertices );
    	clouds.push_back(testcloud);
    	clouds.push_back (projectedcloud);
    	
  	  dif = !newtonsimple ( lbeam, multi, mod_variable[mod_index], dif_tol );

      std::cout << "\ncost: " << lbeam.cost[0] << " " << lbeam.cost[1] << "\n"
      <<" width: " << lbeam.poly.width << " " << lbeam.poly.oldwidth << "\n"
      << " height: " << lbeam.poly.height << " " << lbeam.poly.oldheight << "\n"
      << " thickness: " << lbeam.poly.thickness << " " << lbeam.poly.oldthickness <<"\n\n" <<std::endl;

    	max = sqrt ( lbeam.cost[0]*lbeam.cost[0] + lbeam.cost[1]*lbeam.cost[1]) > cost_tol;

      iter++;
    }
  }

  // visualize(testcloud);
  visualize(clouds);

  return 0;
}