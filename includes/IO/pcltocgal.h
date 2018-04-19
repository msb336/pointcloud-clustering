#include "../meshing/includes.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> colorCloud;

/*
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3  Point_3;
*/


std::vector<Point_3> loadsimple( colorCloud::Ptr cloud )
{
    std::vector<Point_3> pointset;
    for ( size_t i = 0; i < cloud->points.size(); i++ )
    {
        Point_3 q ( cloud->points[i].x,
                    cloud->points[i].y,
                    cloud->points[i].z );
        pointset.push_back ( q );
    }
    return pointset;
}

std::vector<Point> loadexact ( colorCloud::Ptr cloud )
{
    std::vector<Point> pointset;
    for ( size_t i = 0; i < cloud->points.size(); i++ )
    {
        Point q ( cloud->points[i].x,
                    cloud->points[i].y,
                    cloud->points[i].z );
        pointset.push_back ( q );
    }
    return pointset;
}
