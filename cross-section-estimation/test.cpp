#include "includes.h"

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
  bool newtonsimple ( BeamCost &beam, float multiplier, float tolerance=0.1 )
{


  float dcost1 (beam.cost[0] - beam.oldcost[0]);
  float dcost2 (beam.cost[1] - beam.oldcost[1]);


  float dw = beam.poly.width - beam.poly.oldwidth;
   beam.poly.oldwidth = beam.poly.width;
   beam.poly.width = beam.poly.width - multiplier*beam.cost[0]*dw/dcost1;


  float dh = beam.poly.height - beam.poly.oldheight;
  beam.poly.oldheight = beam.poly.height;
  beam.poly.height = beam.poly.height - multiplier*beam.cost[1]*dh/dcost2;


  beam.poly.maxtest();
	beam.poly.calculatePositions();

	float del = sqrt ( dh*dh +dw*dw );
  std::cout << del <<std::endl;
	return ( del < tolerance ); 
	
}

// void modcost( std::vector<float> &cost, Polygon &simple, pointCloud full_cloud, pointCloud &projected)
// void modcost( float &cost, Polygon &simple, pointCloud full_cloud, pointCloud &projected)
void modcost ( BeamCost &beam, pointCloud &projected)
{
  // std::vector<float> costy;
  // Polygon z = beam.poly;
  // NormalCloud x = beam.ncloud;
  // for (int i=0; i<2;i++){costy.push_back(i);}
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
  float planar_tolerance			=	stof ( parameters[1] );
  float cost_tol              = stof ( parameters[2] );
  float dif_tol               = stof ( parameters[3] );
  float multi                 = stof ( parameters[4] );
  float radius                = stof ( parameters[5] );
  int 	minclust 			        = 	1;
  pointCloud::Ptr full_cloud 	    = 	loadcloud ( fullfile );

//////////// Create beam object ///////////
	 Polygon poly (5, 3, 1, Eigen::Vector3f (-15.24, 0, 5.762) );
   poly.makeCloud();


  ////////////// Determine Plane coefficinets for polygon //////////////////
 	pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients );
 	pointsToPlane( poly.lines[0].begin, poly.lines[1].begin, poly.lines[2].begin, plane );
 

  //////////////// Calculate Normals ///////////
  NormalCloud::Ptr full_cloud_with_normals ( new NormalCloud );
  Normals::Ptr just_the_normals ( new Normals );
  computenormals(full_cloud, *just_the_normals, *full_cloud_with_normals, radius);
  std::cout << "number of normals: " << just_the_normals->size() << std::endl;
  visualize ( full_cloud, just_the_normals );


  ///////// Find Points within tolerance of plane ///////////////////s
  NormalCloud::Ptr plane_cloud_with_normals ( new NormalCloud );
  pointsnearplane ( poly, *full_cloud_with_normals, *plane_cloud_with_normals );
  std::cout << plane_cloud_with_normals->size() << " " << full_cloud->size() << std::endl;

  /////////////// Build beam cost object /////////////
  std::vector<float> initial_cost;
  for(int i=0;i<2;i++){initial_cost.push_back(5);}
  BeamCost lbeam ( poly, plane_cloud_with_normals, initial_cost);
  
  pointCloud::Ptr projectedcloud ( new pointCloud) , testcloud ( new pointCloud );
  pcl::copyPointCloud ( *plane_cloud_with_normals, *projectedcloud );
  pcl::copyPointCloud ( *projectedcloud, *testcloud );

  ////////////// Initialize cost function ////////////////////
    bool dif = true;
    bool max = true;

    std::vector<pointCloud::Ptr> clouds;

    while ( dif && max)
    {
    	modcost ( lbeam, *projectedcloud );
    	lbeam.poly.makeCloud();
    	std::cout << "\ncost: " << lbeam.cost[0] << " " << lbeam.cost[1] << " width: " << lbeam.poly.width << " height: " << lbeam.poly.height << " thickness: " << poly.thickness << "\n\n" <<std::endl;
    	clouds.clear();
    	clouds.push_back(testcloud);
    	clouds.push_back (projectedcloud);
    	visualize(clouds );
		  dif = !newtonsimple ( lbeam, multi, dif_tol );
    	max = sqrt ( lbeam.cost[0]*lbeam.cost[0] + lbeam.cost[1]*lbeam.cost[1]) > cost_tol;
      // max = cost > cost_tol;
    	

    }

    return 0;
}