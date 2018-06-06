
#include "../segmentation/includes.h"
#include <pcl/surface/mls.h>
typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::PointNormal> NormalCloud;
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointN;
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
    PointCloud::Ptr vertices;
    std::vector<Eigen::Vector3f> normalpoints;

    Polygon ( float, float, float,  Eigen::Vector3f, float, float );
    Polygon ();
    void calculatePositions();
    void maxtest();
    void makeCloud();
    float rewrite ( float, float);
    void set ( float, float, float,  Eigen::Vector3f, float, float);
};
  Polygon::Polygon(){}
  void Polygon::calculatePositions ( )
  {
    Eigen::Transform<float,3, Eigen::Affine> trans1  ( Eigen::AngleAxisf ( theta, Eigen::Vector3f ( 1,0,0 ) ) );
    Eigen::Transform<float, 3, Eigen::Affine> trans2 ( Eigen::AngleAxisf (phi,    Eigen::Vector3f ( 0,1,0 ) ) );
    Eigen::Transform<float, 3, Eigen::Affine> fulltrans = trans1*trans2;

    std::vector<Eigen::Vector3f> polyObject;
    polyObject.push_back ( placement );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0,width,0 ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0, width, thickness * sign(height) ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0, thickness*sign(width),  thickness*sign(height) ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0, thickness*sign(width), height ) ) );
    polyObject.push_back ( fulltrans*(placement + Eigen::Vector3f( 0, 0, height ) ) );
    // std::cout << "New Cross Section co-ordinates: \n";
    lines.clear();
    normalpoints.clear();
    for (int i = 0; i < polyObject.size(); i++)
    {
      linesegment l;
      l.line  = polyObject[(i+1) % polyObject.size() ] - polyObject[i];
      Eigen::Vector3f cross = l.line.cross(Eigen::Vector3f(-1,0,0));
      normalpoints.push_back( cross / cross.norm() );
      // std::cout << "( " << l.line[0] << " " << l.line[1] << " " << l.line[2] << " )"
      // <<"Built by " << i << " and " << (i+1) % polyObject.size() << " has normal: (" 
      // << datastream[i][0] << " " << datastream[i][1] << " " << datastream[i][2] << ")" << std::endl;


      l.begin = polyObject[i];
      l.end   = polyObject[i+1];
      l.norm  = l.line.norm();
      lines.push_back(l);
    }
  }
  void Polygon::set (float h1, float w, float t, Eigen::Vector3f init, float th = 0, float ph =0 )
  {
    oldthickness=thickness; oldtheta=theta;oldphi=phi;oldheight=height;oldwidth=width;
    height = h1; width = w; thickness = t; placement = init; theta = th; phi = ph;

    // std::cout << "L Beam with cross section properties: \n"
    // << "thickness: " << thickness << "\n"
    // << "width: "    << width << "\n"
    // << "height 1: " << height<<"\n"
    // << "theta: "    <<theta  << "\n"
    // << "phi: "      << phi  << "\n"
    // <<std::endl;

    calculatePositions ();
  }
  Polygon::Polygon ( float h1, float w, float t, Eigen::Vector3f init, float th = 0, float ph =0 )
  {
    set ( h1, w, t, init, th, ph );
  }
  void Polygon::maxtest ()
  {
    if (fabs(thickness) > 100){thickness=100*sign(thickness);}
    else if (fabs(thickness) < 0.25){thickness=0.25;}

    if (fabs(height) > 100){height=100*sign(height);}
    else if (fabs(height) < 0.25){height=0.25;}

    if (fabs(width) > 100){width=100*sign(width);}
    else if (fabs(width) < 0.25){width=0.25;}
  }
  float Polygon::rewrite ( float cost, float oldcost)
  {
    float dcost = cost - oldcost;
   float dthick = thickness - oldthickness;
   float dtheta = theta - oldtheta;
   float dphi = phi - oldphi;
   float dh1  = height - oldheight;
   float dw  = width - oldwidth;
   // std::cout << "\nCost Function stuff:\n"
   // <<"cost: " << cost << " old cost: " << oldcost << " change: " << dcost <<"\n" << std::endl;
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
    
    // std::cout << "Individual change:\n"
    // << "thickness: " << oldthickness << " " << thickness << " " << dthick << " change " << cost/((dcost)/dthick) <<"\n"
    // << "width: "    <<  oldwidth << " " << width << " " << dw << " change " << cost/((dcost)/(dw)) <<"\n"
    // << "height 1: " <<  oldheight <<" "<< height<<" " << dh1 << " change " << cost/((dcost)/(dh1)) <<"\n"
    // << "theta: "    << oldtheta    <<" " <<theta  <<" " << dtheta << " change " << cost/((dcost)/(dtheta)) <<"\n"
    // << "phi: "      << oldphi     << " " << phi   <<" " << dphi << " change " << cost/((dcost)/(dphi)) <<"\n"
    // <<"del norm: "  << del << " dcost " << dcost << std::endl;

    calculatePositions();
    return del;
  }
  void Polygon::makeCloud()
  {
    PointCloud::Ptr temp ( new PointCloud );
    // std::cout << "writing points" << std::endl;
    for (int i = 0; i < lines.size(); i++)
    {
      PointT p1 (lines[i].begin[0],lines[i].begin[1], lines[i].begin[2] );
      temp->push_back ( p1 );
      PointT p2 (lines[i].end[0],lines[i].end[1], lines[i].end[2] );
      temp->push_back ( p2 );

      for ( float j = 0.1; j < lines[i].norm; j += 0.1 )
      {
        // std::cout << j << std::endl;
        Eigen::Vector3f vecform = j*(lines[i].line / lines[i].norm) + lines[i].begin;
        PointT p (vecform[0], vecform[1], vecform[2] );
        temp->push_back ( p );
      }
    }
    // std::cout << "done. " <<std::endl;
    vertices = temp; 
  }

  //////////////////////////I/O/////////////////

  void load_set ( std::string normalfile, NormalCloud &points_with_normals )
  {
    ifstream datastream (normalfile);

    std::vector<std::string> values ;
    // std::vector<pcl::PointNormal> point_set;
    char y;
    std::string i;
    while (datastream >> y) 
    {
      if ( y == ' ' || y == ',' || y == '\t' )
        datastream  >> y ;
      i += y ;
      if ( datastream.peek () == ' ' || datastream.peek () == ',' )
      {
        values.push_back(i);
        i = "";
      }

      if ( datastream.peek () == '\n' )
      {
        values.push_back(i);
        pcl::PointNormal point;
        point.normal_x  = -stof(values[0]);
        point.normal_y  = -stof(values[1]);
        point.normal_z  = -stof(values[2]);
        point.x         = stof(values[3]);
        point.y         = stof(values[4]);
        point.z         = stof(values[5]);
        // point_set.push_back ( point );
        points_with_normals.push_back(point);
        values.clear () ;
        i = "";
      }
    }

  }


///////////////////////// Find Points relevant to 2D plane /////////////////////////////////
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

template<typename CloudType>
void pointsnearplane ( Polygon poly, CloudType &cloud, CloudType &newcloud, float tolerance = 0.02 )
{ 
  newcloud.points.clear();
  std::vector<int> index;
  pcl::ModelCoefficients::Ptr plane ( new pcl::ModelCoefficients);
  pointsToPlane( poly.lines[3].begin, poly.lines[4].begin, poly.lines[5].begin, plane );
    
  for ( int i =0; i < cloud.points.size(); i++ )
  {
    float dist = 
      plane->values[0] * cloud.points[i].x + plane->values[1] * cloud.points[i].y + 
      plane->values[2] * cloud.points[i].z + plane->values[3];
      /*
      if ( cloud.points[i].x < 3)
      {
      std::cout << "Point: (" << cloud.points[i].x << " is " << fabs(dist) 
        << " away from plane. Tolerance: " << tolerance << std::endl;}
        */

    if ( fabs(dist) < tolerance )
    {  newcloud.points.push_back(cloud.points[i]); }
  }

  // std::cout << "Plane Coefficients: " 
  // << plane->values[0] << " " << plane->values[1] << " " << plane->values[2] << " " << plane->values[3] << std::endl;
}


///////////////////////////////// Projecting Points to Polygon /////////////////////////////////////////////
Eigen::Vector3f fitProjection (Eigen::Vector3f v, linesegment l, Eigen::Vector3f &projection)
{
    Eigen::Vector3f pointvector =  v - l.begin;
    float dotproduct = pointvector.dot(l.line / l.norm);
    // std::cout << "\n\n\n\n" << l.line << "\n\n" << l.line / l.norm <<  "\n\n\n\n\n" << std::endl;
    projection = dotproduct * ( l.line / l.norm );
    Eigen::Vector3f bdist = ( l.line - projection);

    
    if ( (projection.norm() + bdist.norm() - l.norm) <= 0.001)
    {
      projection = projection + l.begin;
    }    
    else if ( projection.norm() < bdist.norm() )
    {
      projection = l.begin;
    }
    else
    {
      projection = l.end;
    }

}
float crossnorm ( Eigen::Vector3f a, Eigen::Vector3f b)
{
  float x = a[1]*b[2] - b[1]*a[2];
  float y = -(a[0]*b[2] - b[0]*a[2]);
  float z = a[0]*b[1] - b[0]*a[1];

  return sqrt ( x*x +y*y + z*z );
}
float dotprod ( Eigen::Vector3f a, Eigen::Vector3f b )
{

}
void projectToPolygon ( std::vector<float> &cost, Polygon poly, PointN oldp, PointT &newp, float similarity=0.5 )
{
  float distance = 10000;
  Eigen::Vector3f projection;
  Eigen::Vector3f old_vector( oldp.x, oldp.y, oldp.z );
  Eigen::Vector3f normal_vec ( oldp.normal_x, oldp.normal_y, oldp.normal_z );
  Eigen::Vector3f dv ( 0,0,0 );
  Eigen::Vector3f dot;
  Eigen::Vector3f distance_vector (0,0,0);
  Eigen::Vector3f unit_normal_vec = normal_vec / normal_vec.norm();
  int index = 0;
  for ( int i =0; i < poly.lines.size(); i++ )
  {
    

    float dot = unit_normal_vec.dot( poly.normalpoints[i] );
    if ( dot > similarity )
    {
      
      fitProjection(old_vector, poly.lines[i], projection );
      distance_vector[1] = projection[1] - old_vector[1];
      distance_vector[2] = projection[2] - old_vector[2];
      distance_vector[0] = 0;
      float newdist = distance_vector.norm();
      if (newdist < distance )
      { 
          index = i;
          newp.x = projection[0]; newp.y = projection[1]; newp.z = projection[2];
          distance = newdist;
          dv = distance_vector;
          if ( distance < 0.001 )
          {
            break;
          }
      }
    }

  }

  cost[0] += dv[1]; cost[1] += dv[2];
   std::cout << "( " << old_vector[0] << " " << old_vector[1] << " " << old_vector[2] 
    << ") with normal ( " << unit_normal_vec[0] << " " << unit_normal_vec[1]  << " " << unit_normal_vec[2] << " )\n"
    << " cast to ( " << newp.x << " " << newp.y << " " << newp.z << " " 
    << " ) with normal ( " <<  poly.normalpoints[index][0] << " " << poly.normalpoints[index][1] << " " << poly.normalpoints[index][2] << " )"
    << "\ndistance: " << distance_vector[1] << " " << distance_vector[2]
    <<" cost: " <<  cost[0] << " " << cost[1] << "\n" << std::endl;

  
}


///////////// Normal calculations //////////////////

void computedatastream ( PointCloud::Ptr cloud, Normals &cloud_datastream, NormalCloud &normalcloud, float rad=0.3 )
{
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimate;
  normal_estimate.setInputCloud ( cloud );

  pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> () );
  normal_estimate.setSearchMethod ( tree );

  // Normals::Ptr cloud_datastream ( new Normals );

  normal_estimate.setRadiusSearch ( rad );

  normal_estimate.compute ( cloud_datastream );

  pcl::concatenateFields ( *cloud, cloud_datastream, normalcloud );

}


////////////// Visualize cloud datastream //////////////////////
void visualize ( std::vector<PointCloud::Ptr> cloudset )
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

void visualize ( PointCloud::Ptr cloud , Normals::Ptr datastream)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud, "Single Cloud");
    viewer->addPointCloudNormals<PointT, pcl::Normal> (cloud, datastream, 10, 0.5, "datastream");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Single Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void visualize ( NormalCloud::Ptr cloud)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointN> (cloud, "Single Cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Single Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void mlsdatastream(PointCloud::Ptr &cloud, Normals &just_datastream, NormalCloud::Ptr &cloud_with_datastream, float searchRadius=0.03 )
{
  pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> );
  pcl::MovingLeastSquares<PointT, PointN> mls;

  mls.setComputeNormals ( true );
  mls.setInputCloud ( cloud );
  mls.setPolynomialOrder ( 3 );
  mls.setSearchMethod ( tree );
  mls.setSearchRadius (searchRadius );
  mls.process ( *cloud_with_datastream);

  just_datastream.width    = cloud_with_datastream->width;
  just_datastream.height   = cloud_with_datastream->height;
  just_datastream.is_dense = cloud_with_datastream->is_dense;
  just_datastream.points.resize (just_datastream.width * just_datastream.height);

  for (int i = 0; i < cloud_with_datastream->points.size(); i++ )
  { 
    float nx = cloud_with_datastream->points[i].normal_x;
    float ny = cloud_with_datastream->points[i].normal_y;
    float nz = cloud_with_datastream->points[i].normal_z;
    float norm = sqrt( nx*nx + ny*ny + nz*nz );

    just_datastream.points[i].normal_x = nx / norm;
    just_datastream.points[i].normal_y = ny / norm;
    just_datastream.points[i].normal_z = nz / norm;


  }
}

