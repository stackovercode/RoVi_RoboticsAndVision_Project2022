#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
// frhag@mmmi.sdu.dk
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>

void voxelGrid( pcl::PCLPointCloud2::Ptr input_cloud, pcl::PCLPointCloud2::Ptr &output_cloud )
{
  // create filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*output_cloud);

}


void outlierRemoval( pcl::PCLPointCloud2::Ptr input_cloud, pcl::PCLPointCloud2::Ptr &output_cloud )
{
  //creat the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (200);
  sor.setStddevMulThresh (0.01);
  sor.filter (*output_cloud);

}

void spatialFilter( pcl::PCLPointCloud2::Ptr input_cloud, pcl::PCLPointCloud2::Ptr &output_cloud, int limMin, int limMax)
{
  //create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  //pass.setFilterLimits (260.0, 320.0); //Start
  //pass.setFilterLimits (-87, -80); //Looks promesing
  pass.setFilterLimits (limMin, limMax);
  pass.filter (*output_cloud);
}


int main (int argc, char** argv)
{
  int lowLim = 0;
  int highLim = 0;
  while (true)
  {
  
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
  
  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("/home/rovi2022/projects/RoVi_Project/DenseStereo/src/build/cloud.pcd", *cloud); 
  //reader.read ("../cloud.pcd", *cloud); // Remember to download the file first!
  //reader.read ("/home/rovi2022/projects/RoVi_Project/DenseStereo/src/build/cloud.pcd", *cloud);
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  voxelGrid( cloud, cloud_filtered );

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
       
  outlierRemoval( cloud_filtered, cloud_filtered );
  //int lowLim, highLim;
  char select;
  std::cout << "\nLast limits where: " << lowLim << " to " << highLim << std::endl;
  std::cout <<  "select Lower limit ";
  std::cin >> lowLim;
  std::cout <<  "select higher limit ";
  std::cin >> highLim;

  spatialFilter( cloud_filtered, cloud_filtered ,lowLim,highLim);
 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
  << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write ("cloud_filtered.pcd", *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity(), false);
  std::cout << "Where the lower: " << lowLim << " and high limit: " << highLim << "\n Was it good enough [Y]es? or else any button to try agian?: ";
  std::cin >> select;
  if (toupper(select) == 'Y'){
      break;
  } else {
    std::cout << "Going agian" << std::endl;
  }
  } //End of while
  std::cout << "Ending processing" << std::endl;
  return (0);
}

