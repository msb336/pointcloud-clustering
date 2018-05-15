#include "../segmentation/includes.h"

typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::PointNormal> pointNormal;
typedef pcl::PointXYZ PointT;

struct lbeam
{

  float relative_theta;
  float relative_phi;

	float t;
	float l;
	float h;
	float w;

	float t_certainty;
	float l_certainty;
	float h_certainty;
	float w_certainty;

  std::vector<PointT> polygon;
};
struct linesegment
{
    Eigen::Vector3f line;
    Eigen::Vector3f begin;
    Eigen::Vector3f end;
    float norm;
};

class Polygon
{
  public:
    std::vector<linesegment> lines;
    Polygon ( std::vector<PointT> );
};
  Polygon::Polygon ( std::vector<PointT> polyObject )
  {
    for (int i = 0; i < polyObject.size(); i++)
    {
      linesegment l;
      l.line  = polyObject[i].getArray3fMap() - polyObject[i+1].getArray3fMap();
      l.begin = polyObject[i].getArray3fMap();
      l.end   = polyObject[i+1].getArray3fMap();
      l.norm  = sqrt(l.line[0]*l.line[0]+l.line[1]*l.line[1]+l.line[2]*l.line[2]);
      lines.push_back(l);
    }
  }

void pointsToPlane( const PointT &a, const PointT &b, const PointT &c, const pcl::ModelCoefficients::Ptr &plane )
{
  Eigen::Hyperplane<float,3> eigen_plane = Eigen::Hyperplane<float,3>::Through(
    a.getArray3fMap(), 
    b.getArray3fMap(), 
    c.getArray3fMap());
  plane->values.resize(4);


  for ( size_t i = 0; i < plane->values.size(); i++ )
  {
    plane->values[i] = eigen_plane.coeffs()[i];
  }
}

Eigen::Transform<float,3, Eigen::Affine> rotateToOrigin ( pcl::ModelCoefficients:: Ptr &plane )
{
  Eigen::Vector3f cloud_plane_normal_vector;
  cloud_plane_normal_vector[0] = plane->values[0];
  cloud_plane_normal_vector[1] = plane->values[1];
  cloud_plane_normal_vector[2] = plane->values[2];

  Eigen::Vector3f xy_plane_normal_vector;
  xy_plane_normal_vector[0] = 0;
  xy_plane_normal_vector[1] = 0;
  xy_plane_normal_vector[2] = 1;

  Eigen::Vector3f rotation_vector = cloud_plane_normal_vector.cross (xy_plane_normal_vector);
  Eigen::Vector3f transformVector;
  float length=sqrt(rotation_vector[0]*rotation_vector[0]+rotation_vector[1]*rotation_vector[1]+rotation_vector[2]*rotation_vector[2]);

  if ( length < 0.01 )
  {
    transformVector(0) = 0;
    transformVector(1) = 0;
    transformVector(2) = 0;
  }
  else
  {
    std::cout << "rotation vector " << rotation_vector << std::endl;
    transformVector(0) = rotation_vector[0] / length;
    transformVector(1) = rotation_vector[1] / length;
    transformVector(2) = rotation_vector[2] / length;
  }


  std::cout << "transform vector " << transformVector << " length " << length << std::endl;

  Eigen::Affine3f transformModel = Eigen::Affine3f::Identity();

  float theta = -acos(cloud_plane_normal_vector[0]*xy_plane_normal_vector[0]+cloud_plane_normal_vector[1]*xy_plane_normal_vector[1]+cloud_plane_normal_vector[2]*xy_plane_normal_vector[2]);
  transformModel.rotate (Eigen::AngleAxisf (theta, transformVector));

  Eigen::Transform<float,3, Eigen::Affine> t (Eigen::AngleAxisf ( theta, transformVector) );
  return t;

}
void calculateExtreme ( std::vector<PointT> poly, float &minX, float &maxX, float &minY, float &maxY )
{
  for (int i = 0; i < poly.size(); i++ )
  {
    if ( poly[i].x < minX )
      minX = poly[i].x;
    if ( poly[i].x > maxX )
      maxX = poly[i].x;
    if ( poly[i].y < minY )
      minY = poly[i].y;
    if ( poly[i].y > maxY )
      maxY = poly[i].y;
  }
}

bool simplePolyCheck( PointT p, float minX, float maxX, float minY, float maxY )
{
  std::cout << "simplePolyCheck of point " << p;
  if ( p.x < minX || p.x > maxX || p.y < minY || p.y > maxY )
  {
    std::cout << " returned false" << std::endl;
    return false;
  }

  else
  {
    std::cout << " returned true" << std::endl;
    return true;
  }


}

bool polygonTest ( std::vector<PointT> poly, PointT p, float tolerance = 0)
{
  bool c = false;
  int i, j;
  for ( i = 0, j = poly.size()-1; i < poly.size(); j=i++)
  {
    if ( ((poly[i].y > p.y) != (poly[j].y > p.y)) && 
          (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x) )
      c = !c;
  }
  std::cout << "polygonTest of point " << p << " returned " << c <<std::endl;
  return c;
}

PointT transformPoint (const PointT &point, const Eigen::Transform<float, 3, Eigen::Affine> &transform)
 {
  //ret.getVector3fMap () = transform * point.getVector3fMap ();
  PointT ret = point;
  ret.x = static_cast<float> (transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3));
  ret.y = static_cast<float> (transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3));
  ret.z = static_cast<float> (transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3));
  return ret;
 }


bool inPoly( PointT p, std::vector<PointT> poly, PointT pflat, std::vector<PointT> polyflat, float tolerance )
{
    float minX, maxX, minY, maxY;
    calculateExtreme ( polyflat, minX, maxX, minY, maxY );
    if ( !simplePolyCheck ( pflat, minX-tolerance, maxX+tolerance, minY-tolerance, maxY+tolerance) )
    {
      return false;
    }
    else
    {
      return polygonTest(polyflat, pflat, tolerance);
    }
}


std::vector<int> pointsInPoly ( pointCloud cloud, std::vector<PointT> poly, float tolerance = 0.02 )
{
  std::vector<int> index_points;
  pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients);
  pointsToPlane( poly[0], poly[1], poly[2], plane );

  Eigen::Transform<float,3, Eigen::Affine>  translation = rotateToOrigin ( plane );
  std::cout << "translation matrix: " << translation.matrix() << std::endl;
  std::vector<PointT> polyflat = poly;
  std::cout << "Rotating polygon to origin plane" << std::endl;
  for ( int i = 0; i < poly.size(); i++ )
  {
    polyflat[i] = transformPoint ( poly[i], translation);
    std::cout << poly[i] << " -> " << polyflat[i] << std::endl;
  }

  for ( int i =0; i < cloud.points.size(); i++ )
  {
    float dist = plane->values[0] * cloud.points[i].x + plane->values[1] * cloud.points[i].y + plane->values[2] * cloud.points[i].z - plane->values[3];
    std::cout << "Distance from plane " << dist << std::endl;
    if ( abs(dist) > tolerance )
    { std::cout << cloud.points[i] << " not in plane" << std::endl;}
    else
    { 

      PointT pflat = transformPoint (cloud.points[i], translation);
      std::cout << "rotated " << cloud.points[i] << " to " << pflat << std::endl;
      if ( inPoly ( cloud.points[i], poly, pflat, polyflat, tolerance ) )
      {index_points.push_back(i);
        cout <<"points in poly"<<std::endl;}
    }
      
  }
  return index_points;
}

void projectToPolygon ( float &cost, Polygon &poly, PointT &oldp, PointT &newp )
{

  float distance = 100;
  Eigen::Vector3f projection;
  Eigen::Vector3f old_vector = oldp.getArray3fMap();
  for ( int i =0; i < poly.lines.size(); i++ )
  {
    Eigen::Vector3f pointvector =  old_vector - poly.lines[i].begin;
    float dotproduct = pointvector.dot(poly.lines[i].line / poly.lines[i].norm);
    projection = dotproduct * poly.lines[i].line / poly.lines[i].norm;
    if ( poly.lines[i].norm < projection.norm() )
    {
      projection = poly.lines[i].end;
    }
    else
    {
      projection += poly.lines[i].begin;
    }
    
    Eigen::Vector3f distance_vector = old_vector - projection;
    float newdist = distance_vector.norm();
    std::cout << "Distance from point " << oldp << " = " << newdist << std::endl;

    if (newdist < distance )
    {   std::cout << "casting " << newp << " to (" << projection[0] << ", " << projection[1] << ", " << projection[2] << ")" << std::endl;
        newp.x = projection[0]; newp.y = projection[1]; newp.z = projection[2]; 
        distance = newdist; 
        if ( newdist < 0.001 )
        {
          break;
        }
    }
  }
  std::cout << "\n\n\n\n\n\n" << std::endl;
  cost+=distance;
}

void costfunction ( float &cost, std::vector<PointT> poly, pointCloud &cloud, pointCloud &newcloud, float tolerance = 0.02 )
{
  cost = 0;
  std::vector<int> index_points;
  pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients);
  pointsToPlane( poly[0], poly[1], poly[2], plane );

  Polygon polysegs ( poly );
  for ( int i =0; i < cloud.points.size(); i++ )
  {
    float dist = plane->values[0] * cloud.points[i].x + plane->values[1] * cloud.points[i].y + plane->values[2] * cloud.points[i].z - plane->values[3];

    if ( abs(dist) > tolerance )
    { std::cout << cloud.points[i] << " not in plane" << std::endl;}
    else
    {
      projectToPolygon ( cost, polysegs, cloud.points[i], newcloud.points[i]); }
  }
}

pcl::ModelCoefficients::Ptr getLargestPlane ( pointCloud::Ptr cloud,  float distance )
{

  pcl::ExtractIndices<PointT> extract;
  pcl::SACSegmentation<PointT> seg;
  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold ( distance );

    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );

    seg.setInputCloud ( cloud );
    seg.segment ( *inliers, *coefficients );
    coefficients->values.resize (4) ;

  return coefficients;
}

void fit_beam ( pointCloud::Ptr cloud , lbeam &beamModel)
{
  pcl::ModelCoefficients::Ptr planeModel = getLargestPlane ( cloud, 0.1 );

  std::cout << "Plane Model Coefficients" << std::endl;
  for (int i = 0; i < 4; i ++)
  {
    std::cout << planeModel->values[i] <<   " ";
  }
  std::cout << std::endl;
}

void visualize ( pointCloud::Ptr cloud , Normals::Ptr normals)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud, "Single Cloud");
    viewer->addPointCloudNormals<PointT, pcl::Normal> (cloud, normals, 10, 0.5, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Single Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main ( int argc, char** argv)
{
  std::stringstream readfile ( argv[1] );
  std::vector<std::string> parameters = readparameters ( readfile.str() );

  std::string fullfile    	=   parameters[0];
  // float radius 			  	=	stof ( parameters[1] );
  float tolerance			=	stof ( parameters[1] );
  int 	minclust 			= 	1;
  pointCloud::Ptr cloud 	= 	loadcloud ( fullfile );
  pointCloud::Ptr newcloud ( new pointCloud );
  newcloud = cloud;

 PointT c1(0,0,0), c2(5,0,0), c3(5,5,0), c4(0,5,0);
 std::vector<PointT> poly;

  poly.push_back ( c1 );
  poly.push_back ( c2 );
  poly.push_back ( c3 );
  poly.push_back ( c4 );


  float cost = 0;

  std::cout << "Moving points to polygon and calculating cost " << std::endl;
  costfunction ( cost, poly, *cloud, *newcloud );
  std::cout << "Total cost: " << cost << std::endl;
  return 0;
}
