#include "includes.h"

bool newtonsimple ( float cost, float &oldcost, Polygon &p, float tolerance = 0.1 )
{
	float dcost = cost - oldcost;
	oldcost = cost;
	float dw = p.width - p.oldwidth;
	p.oldwidth = p.width;
	p.width = p.width - cost*dw/dcost;

	float dh = p.height - p.oldheight;
	p.oldheight = p.height;
	p.height = p.height - cost*dh/dcost;

	float dt = p.thickness - p.oldthickness;
	p.oldthickness = p.thickness;
	p.thickness = p.thickness - 0.25*cost*dt/dcost;
	p.calculatePositions();
	float del = sqrt ( dt*dt + dw*dw + dh*dh );
	return ( del < tolerance ); 
	
}

void modcost( float &cost, Polygon &simple, pointCloud cloud, pointCloud &projected)
{
	cost = 0;
	for (int i = 0; i < cloud.size(); i++)
	projectToPolygon ( cost, simple, cloud[i], projected[i]);
}
int main ( )
{
  std::vector<std::string> parameters = readparameters ( "../parameters.txt" );

  std::string fullfile    	=   parameters[0];
  // float radius 			  	=	stof ( parameters[1] );
  float tolerance			=	stof ( parameters[1] );
  float cost_tol = stof ( parameters[2] );
  float dif_tol = stof ( parameters[3] );
  int 	minclust 			= 	1;
  pointCloud::Ptr cloud 	= 	loadcloud ( fullfile );


	 Polygon poly (5, 3, 1, Eigen::Vector3f (0,0,0) );
	 poly.makeCloud();
	 pointCloud::Ptr vv ( new pointCloud);
	 *vv+=poly.vertices;

	 visualize(vv);
	 for (int i = 0; i < poly.lines.size(); i++)
	 {
	 	std::cout << "begin: "
	 	<< poly.lines[i].begin[0] << " " << poly.lines[i].begin[1] << " " << poly.lines[i].begin[2] << "\nend: "
	 	<< poly.lines[i].end[0] << " " << poly.lines[i].end[1] << " " << poly.lines[i].end[2] << "\nvector: "
	 	<< poly.lines[i].line[0] << " " << poly.lines[i].line[1] << " " << poly.lines[i].line[2] << " \nnorm: "
	 	<< poly.lines[i].norm << std::endl;
 	 }

 	pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients );
 	pointsToPlane( poly.lines[0].begin, poly.lines[1].begin, poly.lines[2].begin, plane );
  std::cout
  << "plane values: " << plane->values[0] << " " << plane->values[1] << " " << plane->values[2] << " " << plane->values[3]
    << std::endl;

    pointCloud::Ptr testcloud ( new pointCloud );
    
    pointsnearplane ( poly, *cloud, *testcloud );
	pointCloud::Ptr projectedcloud ( new pointCloud );
	*projectedcloud += *testcloud;
	
    float cost;
    float oldcost = 5;
    bool dif = true;
    bool max = true;
    pointCloud::Ptr vertexcloud ( new pointCloud);
    std::vector<pointCloud::Ptr> clouds;
    while ( dif && max)
    {

    	modcost ( cost, poly, *testcloud, *projectedcloud );
    	poly.makeCloud();
    	vertexcloud->points.clear();
    	*vertexcloud+=poly.vertices;
    	std::cout << "cost: " << cost << " width: " << poly.width << " height: " << poly.height << std::endl;
    	clouds.clear();
    	clouds.push_back(projectedcloud);
    	clouds.push_back (vertexcloud);
    	visualize(clouds);
    	dif = !newtonsimple( cost, oldcost, poly, dif_tol );
    	max = cost > cost_tol;
    	

    }

    return 0;
}