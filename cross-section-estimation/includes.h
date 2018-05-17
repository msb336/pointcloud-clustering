#include "../segmentation/includes.h"

typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::PointNormal> pointNormal;
typedef pcl::PointXYZ PointT;
float sign ( float number )
{
  float ss;
  if ( number >= 0 )
  { ss = 1; }
  else
  { ss = -1;}
  return ss;
}
// Necessary
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

// Necessary
class Polygon
{
  public:
    float oldthickness;
    float oldtheta;
    float oldphi;
    float oldheight;
    float oldwidth;
    

    Eigen::Vector3f placement;
    float thickness=0;
    float theta=0;
    float phi=0;
    float height=0;
    float width=0;
    std::vector<linesegment> lines;
    pointCloud vertices;

    Polygon ( float, float, float,  Eigen::Vector3f, float, float );
    void calculatePositions();
    void maxtest();
    void makeCloud();
    float rewrite ( float, float);
    void set ( float, float, float,  Eigen::Vector3f, float, float);
};
  void Polygon::calculatePositions ( )
  {
    Eigen::Transform<float,3, Eigen::Affine> trans1  ( Eigen::AngleAxisf ( theta, Eigen::Vector3f ( 1,0,0 ) ) );
    Eigen::Transform<float, 3, Eigen::Affine> trans2 ( Eigen::AngleAxisf (phi,    Eigen::Vector3f ( 0,1,0 ) ) );
    Eigen::Transform<float, 3, Eigen::Affine> fulltrans = trans1*trans2;

    std::vector<Eigen::Vector3f> polyObject;
    polyObject.push_back ( placement );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( width, 0,0 ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( width, 0, thickness * sign(height) ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( thickness*sign(width), 0, thickness*sign(height) ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( thickness*sign(width), 0, height ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0, 0, height ) ) );
    // std::cout << "New Cross Section co-ordinates: \n";
    lines.clear();
    for (int i = 0; i < polyObject.size(); i++)
    {
      // std::cout << "( " << polyObject[i][0] << " " << polyObject[i][1] << " " << polyObject[i][2] << " )\n";
      linesegment l;
      l.line  = polyObject[(i+1) % polyObject.size() ] - polyObject[i];
      
        // std::cout << "line between " << polyObject[i] << " and " << polyObject[i+1] << " is " << l.line << std::endl;
      l.begin = polyObject[i];
      l.end   = polyObject[i+1];
      l.norm  = l.line.norm();
      lines.push_back(l);
    }
    // std::cout<<std::endl;
  }
  void Polygon::set (float h1, float w, float t, Eigen::Vector3f init, float th = 0, float ph =0 )
  {
    oldthickness=thickness; oldtheta=theta;oldphi=phi;oldheight=height;oldwidth=width;
    height = h1; width = w; thickness = t; placement = init; theta = th; phi = ph;

    std::cout << "L Beam with cross section properties: \n"
    << "thickness: " << thickness << "\n"
    << "width: "    << width << "\n"
    << "height 1: " << height<<"\n"
    << "theta: "    <<theta  << "\n"
    << "phi: "      << phi  << "\n"
    <<std::endl;

    calculatePositions ();
  }
  Polygon::Polygon ( float h1, float w, float t, Eigen::Vector3f init, float th = 0, float ph =0 )
  {
    set ( h1, w, t, init, th, ph );
  }
  void Polygon::maxtest ()
  {
    if (abs(thickness) > 100){thickness=100*sign(thickness);}
    else if (abs(thickness) < 0.25){thickness=0.25;}

    if (abs(height) > 100){height=100*sign(height);}
    else if (abs(height) < 0.25){height=0.25;}

    if (abs(width) > 100){width=100*sign(width);}
    else if (abs(width) < 0.25){width=0.25;}
  }
  float Polygon::rewrite ( float cost, float oldcost)
  {
    float dcost = cost - oldcost;
   float dthick = thickness - oldthickness;
   float dtheta = theta - oldtheta;
   float dphi = phi - oldphi;
   float dh1  = height - oldheight;
   float dw  = width - oldwidth;
   std::cout << "\nCost Function stuff:\n"
   <<"cost: " << cost << " old cost: " << oldcost << " change: " << dcost <<"\n" << std::endl;
    oldthickness=thickness; oldtheta=theta;oldphi=phi;oldheight=height;oldwidth=width;
      thickness = 
        thickness - (cost/(dcost/dthick));
      theta = 
        theta - (cost/(dcost/dtheta));
      phi = 
        phi - (cost/(dcost/dphi));
      height = 
        height - (cost/(dcost/dh1));
      width = 
        width - (cost/(dcost/dw));
    // maxtest();
    float del = sqrt ( pow(dthick,2) + pow(dtheta,2) + pow(dphi,2) + pow(dh1,2) + pow(dw,2) );
    
    std::cout << "Individual change:\n"
    << "thickness: " << oldthickness << " " << thickness << " " << dthick << " change " << cost/((dcost)/dthick) <<"\n"
    << "width: "    <<  oldwidth << " " << width << " " << dw << " change " << cost/((dcost)/(dw)) <<"\n"
    << "height 1: " <<  oldheight <<" "<< height<<" " << dh1 << " change " << cost/((dcost)/(dh1)) <<"\n"
    << "theta: "    << oldtheta    <<" " <<theta  <<" " << dtheta << " change " << cost/((dcost)/(dtheta)) <<"\n"
    << "phi: "      << oldphi     << " " << phi   <<" " << dphi << " change " << cost/((dcost)/(dphi)) <<"\n"
    <<"del norm: "  << del << " dcost " << dcost << std::endl;

    calculatePositions();
    return del;
  }
  void Polygon::makeCloud()
  {
    vertices.clear();
    // std::cout << "writing points" << std::endl;
    for (int i = 0; i < lines.size(); i++)
    {
      PointT p1 (lines[i].begin[0],lines[i].begin[1], lines[i].begin[2] );
      vertices.push_back ( p1 );
      PointT p2 (lines[i].end[0],lines[i].end[1], lines[i].end[2] );
      vertices.push_back ( p2 );

      for ( float j = 0.1; j < lines[i].norm; j += 0.1 )
      {
        // std::cout << j << std::endl;
        Eigen::Vector3f vecform = j*(lines[i].line / lines[i].norm) + lines[i].begin;
        PointT p (vecform[0], vecform[1], vecform[2] );
        vertices.push_back ( p );
      }
    }
    // std::cout << "done. " <<std::endl;
  }



// Necessary
void pointsToPlane( Eigen::Vector3f &a, Eigen::Vector3f &b, Eigen::Vector3f &c, const pcl::ModelCoefficients::Ptr &plane )
{
  Eigen::Hyperplane<float,3> eigen_plane = Eigen::Hyperplane<float,3>::Through(
      a, 
      b, 
      c);

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


///////////////////////////////////// Test if point inside polygon ////////////////////////////////////
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




///////////////////////////////// Projecting Points to Polygon /////////////////////////////////////////////
void projectToPolygon ( float &cost, Polygon &poly, PointT &oldp, PointT &newp )
{
  int seg;
  float distance = 10000;
  Eigen::Vector3f projection;
  Eigen::Vector3f old_vector = oldp.getArray3fMap();
  for ( int i =0; i < poly.lines.size(); i++ )
  {


    Eigen::Vector3f pointvector =  old_vector - poly.lines[i].begin;
    float dotproduct = pointvector.dot(poly.lines[i].line / poly.lines[i].norm);
    projection = dotproduct * poly.lines[i].line / poly.lines[i].norm;
    Eigen::Vector3f bdist = poly.lines[i].end - projection;

    
    if ( projection.norm() + bdist.norm() - poly.lines[i].norm <= 0.01 && dotproduct > 0)
    {
      projection = projection + poly.lines[i].begin;
    }    
    else if ( projection.norm() < bdist.norm() )
    {
      projection = poly.lines[i].begin;
    }
    else
    {
      projection = poly.lines[i].end;
    }
    
    // projection += poly.lines[i].begin;

    Eigen::Vector3f distance_vector = old_vector - projection;
    float newdist = distance_vector.norm();

    if (newdist < distance )
    { 

      seg = i; 
      newp.x = projection[0]; newp.y = projection[1]; newp.z = projection[2];
        distance = newdist; 
        if ( distance < 0.001 )
        {
          break;
        }
    }
  }

  cost+=distance;
}

void pointsnearplane ( Polygon poly, pointCloud &cloud, pointCloud &newcloud, float tolerance = 0.02 )
{ 
  newcloud.points.clear();
  std::vector<int> index;
  pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients);
  pointsToPlane( poly.lines[3].begin, poly.lines[4].begin, poly.lines[5].begin, plane );

  for ( int i =0; i < cloud.points.size(); i++ )
  {
    float dist = 
      plane->values[0] * cloud.points[i].x + plane->values[1] * cloud.points[i].y + 
      plane->values[2] * cloud.points[i].z - plane->values[3];


    if ( abs(dist) < tolerance )
    {  newcloud.points.push_back(cloud.points[i]); }
  }
}
void costfunction ( float &cost, Polygon poly, pointCloud &cloud, pointCloud &newcloud, float tolerance = 0.02 )
{
  cost = 0;
  newcloud.points.clear(); PointT p;
  for ( int i =0; i < cloud.points.size(); i++ )
  { projectToPolygon ( cost, poly, cloud.points[i], p); newcloud.points.push_back(p);}
}

///////////////////// Shape Modification /////////////////////////////////////////////


////////////// Visualize cloud normals //////////////////////
void visualize ( std::vector<pointCloud::Ptr> cloudset )
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    for (int i =0; i< cloudset.size(); i++)
    {
      viewer->addPointCloud<PointT> (cloudset[i],std::to_string(i) );
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(i));
    }

    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
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