#include <iostream>
#include <vector>

#include <dlib/clustering.h>
#include <dlib/matrix.h>

#include <dlib/pixel.h>
#include <dlib/array2d.h>
#include <dlib/image_transforms.h>
#include <dlib/image_io.h>
#include <dlib/rand.h>



using namespace std;
using namespace dlib;

#include "../segmentation/includes.h"


typedef matrix<double,3,1> pointMatrix;

std::vector<pointMatrix> pclToMatrixVector ( PointCloud::Ptr cloud )
{
    std::vector<pointMatrix> samples;
    pointMatrix m;

   for ( size_t i = 0; i < cloud->points.size(); i++ )
   {
       m(0) = cloud->points[i].x;
       m(1) = cloud->points[i].y;
       m(2) = cloud->points[i].z;
       samples.push_back ( m );
   }
   return samples;
}


void kmeans (std::vector<pcl::PointIndices> &cluster_index, std::vector<pointMatrix> samples, std::string kernel, int num_clusters )
{
    typedef sigmoid_kernel<pointMatrix> sigkernel;
    typedef radial_basis_kernel<pointMatrix> radialkernel;
        
    if ( kernel == "sigmoid")
    {
        kcentroid<sigkernel> kc(sigkernel(), 0.01, 5);
    }
    else
    {
        kcentroid<radialkernel> kc(radialkernel(0.1),0.01, 8);

    }

    

    std::vector<pointMatrix> initial_centers;

   cout << "Number of points: " << samples.size() << endl;

    std::vector<unsigned long> assignments;

    std::cout << "set unsigned long assignments " << std::endl;
    if ( kernel == "sigmoid")
    {
        assignments = spectral_cluster(sigkernel(), samples, num_clusters);
    }
    else
    {
        std::cout << "spectral clustering" << std::endl;
        assignments = spectral_cluster(radialkernel(0.1), samples, num_clusters);
    }

    std::cout << "indexing the clusters" << std::endl;
    for ( size_t list = 0; list < num_clusters; list++ )
    {
        pcl::PointIndices index;
        cluster_index.push_back ( index );
    }

    for (size_t i = 0; i < assignments.size (); i ++ )
    {
        cluster_index[assignments[i]].indices.push_back ( i );
    }
}






