
#include "../segmentation/includes.h"
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
    pointCloud::Ptr vertices;
    std::vector<Eigen::Vector3f> normals;

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
    normals.clear();
    for (int i = 0; i < polyObject.size(); i++)
    {
      linesegment l;
      l.line  = polyObject[(i+1) % polyObject.size() ] - polyObject[i];
      Eigen::Vector3f cross = l.line.cross(Eigen::Vector3f(-1,0,0));
      normals.push_back( cross / cross.norm() );
      // std::cout << "( " << l.line[0] << " " << l.line[1] << " " << l.line[2] << " )"
      // <<"Built by " << i << " and " << (i+1) % polyObject.size() << " has normal: (" 
      // << normals[i][0] << " " << normals[i][1] << " " << normals[i][2] << ")" << std::endl;


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
    pointCloud::Ptr temp ( new pointCloud );
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
      plane->values[2] * cloud.points[i].z - plane->values[3];
      /*
      std::cout << "Point: (" << cloud.points[i].x << " " << cloud.points[i].y << " " <<cloud.points[i].z << " ) "
      << "is " << dist << " away from plane." << std::endl; */

    if ( abs(dist) < tolerance )
    {  newcloud.points.push_back(cloud.points[i]); }
  }

  std::cout << "Plane Coefficients: " 
  << plane->values[0] << " " << plane->values[1] << " " << plane->values[2] << " " << plane->values[3] << std::endl;
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
void projectToPolygon ( std::vector<float> &cost, Polygon poly, PointN oldp, PointT &newp )
{
  float distance = 10000;
  Eigen::Vector3f projection;
  Eigen::Vector3f old_vector( oldp.x, oldp.y, oldp.z );
  Eigen::Vector3f normal_vec ( oldp.normal_x, oldp.normal_y, oldp.normal_z );
  Eigen::Vector3f dv ( 0,0,0 );
  Eigen::Vector3f dot;
  Eigen::Vector3f distance_vector (0,0,0);
  Eigen::Vector3f norm;

  for ( int i =0; i < poly.lines.size(); i++ )
  {
    Eigen::Vector3f unit_normal_vec = normal_vec / normal_vec.norm();
    // cross = poly.normals[i].cross ( unit_normal_vec );
    norm = poly.normals[i] - unit_normal_vec;

    // std::cout << "cross product of ( " << poly.normals[i][0] << " " << poly.normals[i][1] << " " << poly.normals[i][2] << ") and ( "
    //   << unit_normal_vec[0] << " " << unit_normal_vec[1] << " " << unit_normal_vec[2] << ") is:\n"
    //  << "\n" << cross << "\n" << "norm: " << norm.norm() << "\n" << std::endl;

    float dot = unit_normal_vec.dot( poly.normals[i] );
    if ( dot > 0.5 ) //( cn < 0.5 )
    {
      fitProjection(old_vector, poly.lines[i], projection );
      distance_vector = old_vector - projection;
      distance_vector[0] = 0;
      float newdist = distance_vector.norm();
      if (newdist < distance )
      { 


          newp.x = projection[0]; newp.y = projection[1]; newp.z = projection[2];
          distance = newdist;
          dv = distance_vector;
          if ( distance < 0.001 )
          {
            break;
          }
      }
    }
    /*
    else
    {
      std::cout << "cross product of ( " << poly.normals[i][0] << " " << poly.normals[i][1] << " " << poly.normals[i][2] << ") and ( "
       << unit_normal_vec[0] << " " << unit_normal_vec[1] << " " << unit_normal_vec[2] << ") was " << cn << std::endl;
    }
    */
  }

  cost[0] += dv[1]; cost[1] += dv[2];
  // std::cout << "( " << old_vector[0] << " " << old_vector[1] << " " << old_vector[2] << ") cast to ( "
  // << newp.x << " " << newp.y << " " << newp.z << ") distance: << : " << distance_vector[1] << " " << distance_vector[2] << "\n"
  // << "norm: " << norm.norm() << " cost: " <<  cost[0] << " " << cost[1] << "\n" << std::endl;

  /*
  if (dv.norm() == 0)
  {
     std::cout << " Point \n( " 
      << old_vector[0] << " " 
      << old_vector[1] << " " 
      << old_vector[2] << ") with normal couldn't find a home." << std::endl;
  }
  */
  
}


///////////// Normal calculations //////////////////

void computenormals ( pointCloud::Ptr cloud, Normals &cloud_normals, NormalCloud &normalcloud, float rad=0.3 )
{
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimate;
  normal_estimate.setInputCloud ( cloud );

  pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> () );
  normal_estimate.setSearchMethod ( tree );

  // Normals::Ptr cloud_normals ( new Normals );

  normal_estimate.setRadiusSearch ( rad );

  normal_estimate.compute ( cloud_normals );

  pcl::concatenateFields ( *cloud, cloud_normals, normalcloud );

}


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