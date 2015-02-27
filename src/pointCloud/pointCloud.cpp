#include "PointCloud.h"
PointCloud::PointCloud() 
{

}

void PointCloud::init(cv::Mat disp, cv::Mat colorImage)
{
  biggerDisp = cv::Mat::zeros(IMAGEHEIGHT+40,IMAGEWIDTH+40,disp.type());
  biggerLOI = cv::Mat::zeros(IMAGEHEIGHT+40,IMAGEWIDTH+40,colorImage.type());
  disp.copyTo(biggerDisp(cv::Rect(20,20,IMAGEWIDTH,IMAGEHEIGHT)));
  colorImage.copyTo(biggerLOI(cv::Rect(20,20,IMAGEWIDTH,IMAGEHEIGHT)));
}
bool PointCloud::dispToXYZRGB(cv::Rect regionRect)
{
  float            x, y, z; 
  int              r, g, b;
  //int              nPoints = 0;
  int              i, j, k;
  unsigned short   disparity;

  // The format for the output file is:
  // <x> <y> <z> <red> <grn> <blu>
  // <x> <y> <z> <red> <grn> <blu>
  // ...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyzRGBcloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyzCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

  // Determine the number of pixels spacing per row
  for ( i = regionRect.y; i < regionRect.y+regionRect.height; i++ )
  {
      for ( j = regionRect.x; j < regionRect.x+regionRect.width; j++ )
      {
          unsigned short disparity = (unsigned short)biggerDisp.at<uchar>(i, j);

          // do not save invalid points
          if ( disparity>1 && disparity < 255 )
          {
              // convert the 16 bit disparity value to floating point x,y,z
              //triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

              float z = (FOCALLENTH*BASELINE)/(float)disparity;
              float x = (j*z)/FOCALLENTH; //i = row
              float y = (i*z)/FOCALLENTH; //j = col
              // look at points within a range
              b = (int)biggerLOI.at<cv::Vec3b>(i,j)[0];
              g = (int)biggerLOI.at<cv::Vec3b>(i,j)[1];
              r = (int)biggerLOI.at<cv::Vec3b>(i,j)[2];

              pcl::PointXYZRGB pointRGB;
              pcl::PointXYZ point;
              point.x = x;
              point.y = y;
              point.z = z;
              pointRGB.x = x;
              pointRGB.y = y;
              pointRGB.z = z;
              uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
              pointRGB.rgb = *reinterpret_cast<float*>(&rgb);
              point_xyzRGBcloud_ptr->points.push_back(pointRGB);
              point_xyzCloudPtr->points.push_back(point);
          
          }
      }
  }

  point_xyzRGBcloud_ptr->width = (int) point_xyzRGBcloud_ptr->points.size();
  point_xyzRGBcloud_ptr->height = 1;
  point_xyzRGBcloud_ptr->is_dense = false;

  point_xyzCloudPtr->width = (int) point_xyzCloudPtr->points.size();
  point_xyzCloudPtr->height = 1;
  point_xyzCloudPtr->is_dense = false;

  //pcl::io::savePCDFileASCII("out/xyz_point_cloud.pcd",*point_xyzCloudPtr);
  //pcl::io::savePLYFileASCII("output/point_xyzRGBcloud_ptr.ply",*point_xyzRGBcloud_ptr);
  std::cout << "PointCloud before filtering has: " << point_xyzCloudPtr->points.size() << " data points." << std::endl;
  if(point_xyzCloudPtr->points.size())
  {
    // Find center point of point cloud
    Eigen::Vector4f centroid4;
    compute3DCentroid(*point_xyzCloudPtr, centroid4);
    
    clusterCenter3Dpoint.x = centroid4(0);
    clusterCenter3Dpoint.y = centroid4(1);
    clusterCenter3Dpoint.z = centroid4(2);

    int indexMinX=0, indexMaxX=0;
    int indexMinY=0, indexMaxY=0;
    int indexMinZ=0, indexMaxZ=0;
    // Find extreeme points in the convex hull point cloud
    for(int k=0;k<point_xyzCloudPtr->width;k++) 
    { 
      if(point_xyzCloudPtr->points[indexMaxX].x < point_xyzCloudPtr->points[k].x)
      {
        indexMaxX = k; 
      }
      if(point_xyzCloudPtr->points[indexMinX].x > point_xyzCloudPtr->points[k].x)
      {
        indexMinX = k; 
      }
      if(point_xyzCloudPtr->points[indexMaxY].y < point_xyzCloudPtr->points[k].y)
      {
        indexMaxY = k; 
      }
      if(point_xyzCloudPtr->points[indexMinY].y > point_xyzCloudPtr->points[k].y)
      {
        indexMinY = k; 
      }
      if(point_xyzCloudPtr->points[indexMaxZ].z < point_xyzCloudPtr->points[k].z)
      {
        indexMaxZ = k; 
      }
      if(point_xyzCloudPtr->points[indexMinZ].z > point_xyzCloudPtr->points[k].z)
      {
        indexMinZ = k; 
      }
    }

    clusterFront3Dpoint.x = point_xyzCloudPtr->points[indexMinZ].x;
    clusterFront3Dpoint.y = point_xyzCloudPtr->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterFront3Dpoint.z = point_xyzCloudPtr->points[indexMinZ].z;
/*
    cluster3DWidthHeightDepth.x = (point_xyzCloudPtr->points[indexMaxX].x - point_xyzCloudPtr->points[indexMinX].x);
    cluster3DWidthHeightDepth.y = (point_xyzCloudPtr->points[indexMaxY].y - point_xyzCloudPtr->points[indexMinY].y);
    cluster3DWidthHeightDepth.z = (point_xyzCloudPtr->points[indexMaxZ].z - point_xyzCloudPtr->points[indexMinZ].z);

    clusterRight3Dpoint.x = point_xyzCloudPtr->points[indexMaxX].x;
    clusterRight3Dpoint.y = point_xyzCloudPtr->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterRight3Dpoint.z = point_xyzCloudPtr->points[indexMaxX].z;

    clusterLeft3Dpoint.x = point_xyzCloudPtr->points[indexMinX].x;
    clusterLeft3Dpoint.y = point_xyzCloudPtr->points[indexMinY].y + cluster3DWidthHeightDepth.y/2;
    clusterLeft3Dpoint.z = point_xyzCloudPtr->points[indexMinX].z;

    upperLeftCorner3Dpoint.x = point_xyzCloudPtr->points[indexMinX].x;
    upperLeftCorner3Dpoint.y = point_xyzCloudPtr->points[indexMaxY].y;
    upperLeftCorner3Dpoint.z = point_xyzCloudPtr->points[indexMinZ].z+cluster3DWidthHeightDepth.z/2;

    lowerRightCorner3Dpoint.x = point_xyzCloudPtr->points[indexMaxX].x;
    lowerRightCorner3Dpoint.y = point_xyzCloudPtr->points[indexMinY].y;
    lowerRightCorner3Dpoint.z = point_xyzCloudPtr->points[indexMinZ].z+cluster3DWidthHeightDepth.z/2;
    */
   
    /*
    std::stringstream ss;
    ss << "output/clusters/cloud_" << vIt << "_distance_" << clusterCenter3Dpoint.z << ".ply";
    pcl::io::savePLYFileASCII(ss.str (),*point_xyzRGBcloud_ptr); 
    */
    vIt++;
    return true;
  }

  
 // 
 /*
  int vIt = 0;

  // Filter objects futher than 10 meters away
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyzCloudPtr2to8 (new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud (point_xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.0,50.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*point_xyzCloudPtr2to8);
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (point_xyzCloudPtr2to8);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*point_xyzCloudPtr2to8);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2to8 (new pcl::PointCloud<pcl::PointXYZ>);
  // Filter outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (point_xyzCloudPtr2to8);
  //sor.setMeanK (200); //night
  sor.setMeanK (300); //day
  sor.setStddevMulThresh (0.9); 
  sor.filter (*cloud_filtered2to8);

  //std::cout << "PointCloud after filtering has: " << cloud_filtered2to8->points.size ()  << " data points." << std::endl; 

  //pcl::io::savePLYFileASCII("out/cloud_filtered2to8.ply",*cloud_filtered2to8);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered2to8);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.09); // 2cm
  //ec.setMinClusterSize (500); //night
  ec.setMinClusterSize (250); //day
  //ec.setMaxClusterSize (20000); // Night
  ec.setMaxClusterSize (2000); // day
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered2to8);
  ec.extract (cluster_indices);

  */

  return false;
}

cv::Point3f PointCloud::projectFrom3Dto2D(cv::Point3f world3Dcoordinate)
{
    cv::Point3f world2Dcoordiante;
    world2Dcoordiante.x = (world3Dcoordinate.x*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.y = (world3Dcoordinate.y*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.z = (world3Dcoordinate.x*FOCALLENTH)/world2Dcoordiante.x;
    return world2Dcoordiante;
}



