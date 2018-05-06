#include "../includes/dlib_clustering.h"
#include <cmath>


int main (  )
{
    
    std::string pfile = "../parameters.txt";

    std::vector<std::string> parameters = readparameters( pfile );
    std::string filename = parameters[0];

    float leafsize = stof ( parameters[1] );
    int noise_neighbors = stoi ( parameters[2] );
    float noise_std = stof ( parameters[3] );

    std::string cluster_option      =   parameters[4];
    std::string kernel              =    parameters[5] ;
    int num_clust                   =    stoi ( parameters[6] );
    bool vis                        =    stob ( parameters[7] );
    bool save                       =    stob ( parameters[8] );    
    std::string savefile = parameters[9];

    std::cout << "Loading cloud" << std::endl;
    pointCloud::Ptr cloud = loadcloud ( filename );
    std::cout << "Loaded cloud with : " << cloud->points.size() << " points" << std::endl;
    
    std::vector<pcl::PointIndices> cluster_idx ;

    if ( leafsize > 0 )
        downsample ( cloud, leafsize);
    if ( noise_neighbors > 0 )
        noisefilter ( cloud, noise_neighbors, noise_std );
    
    if ( cluster_option == "kmeans")
    {  
        std::vector<pointMatrix> pMat = pclToMatrixVector ( cloud );
        cluster_idx = kmeans ( pMat, kernel, num_clust );
        
    }
    else if ( cluster_option == "bottomup")
    {
        long n = cloud->points.size ();
        matrix<float> dists;

        dists.set_size ( n, n );
        for ( size_t i = 0 ; i < n; i ++ )
        {
            for ( size_t j = 0; j < n; j ++ )
            {
                //2-norm
                dists(i,j) =
                pow(cloud->points[i].x - cloud->points[j].x, 2) +
                pow(cloud->points[i].y - cloud->points[j].y, 2) +
                pow(cloud->points[i].z - cloud->points[j].z, 2);
            }
        }

        std::vector<unsigned long> labels;
        std::cout << "bottom up cluster begin" << std::endl;
        bottom_up_cluster ( dists, labels,  num_clust );
        std::cout << "bottom up cluster end" << std::endl;
        
        int num_clusters = 0;
        pcl::PointIndices index;
        for (size_t i = 0; i < labels.size (); i ++ )
        {
            while ( labels[i] >= cluster_idx.size () )
            {
                cluster_idx.push_back ( index );
            }

            cluster_idx[labels[i]].indices.push_back ( i );
        }
    }

    std::vector<colorCloud::Ptr> segments = split ( cloud, cluster_idx );
    if ( vis == true)
        visualize( segments );

    if ( save == true )
        writetoXYZ ( savefile, segments );

}