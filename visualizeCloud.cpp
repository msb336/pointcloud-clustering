#include "segmentation/includes.h"

main (int argc, char** argv)
{
	pointCloud::Ptr cloud =  loadcloud ( argv[1] );
	visualize (cloud) ;
}