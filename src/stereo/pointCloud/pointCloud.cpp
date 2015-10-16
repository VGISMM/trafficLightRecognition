#include "PointCloud.h"
PointCloud::PointCloud() 
{

}

void PointCloud::init(cv::Mat disp, cv::Mat colorImage)
{
  biggerDisp = disp.clone();
  biggerLOI = colorImage.clone();
}

bool PointCloud::findRoadSurfaceCoefficients()
{
  float            x, y, z; 
  int              r, g, b;
  //int              nPoints = 0;
  int              i, j, k;
  unsigned short   disparity;
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr showpoint_xyzRGBcloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Only creates cloud for lower half of image
  for ( i = 0; i < IMAGEHEIGHT; i++ )
  {
    for ( j = 0; j < IMAGEWIDTH; j++ )
    {
      unsigned short disparity = (unsigned short)biggerDisp.at<uchar>(i, j);
      // do not save invalid points
      if ( disparity>6 && disparity < 255 )
      {
        float z = (FOCALLENTH*BASELINE)/(float)disparity;
        float x = (j*z)/FOCALLENTH; //i = row
        float y = (i*z)/FOCALLENTH; //j = col
        // get color values
        b = (int)biggerLOI.at<cv::Vec3b>(i,j)[0];
        g = (int)biggerLOI.at<cv::Vec3b>(i,j)[1];
        r = (int)biggerLOI.at<cv::Vec3b>(i,j)[2];

        pcl::PointXYZRGB showpointRGB;
        showpointRGB.x = x;
        showpointRGB.y = y;
        showpointRGB.z = z;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        showpointRGB.rgb = *reinterpret_cast<float*>(&rgb);
        showpoint_xyzRGBcloud_ptr->points.push_back(showpointRGB);
      }
    }
  }

  showpoint_xyzRGBcloud_ptr->width = (int) showpoint_xyzRGBcloud_ptr->points.size();
  showpoint_xyzRGBcloud_ptr->height = 1;
  showpoint_xyzRGBcloud_ptr->is_dense = false;
  pcl::io::savePLYFileASCII("showpoint_xyzRGBcloud_ptr.ply",*showpoint_xyzRGBcloud_ptr);
  */

  // The format for the output file is:
  // <x> <y> <z> <red> <grn> <blu>
  // <x> <y> <z> <red> <grn> <blu>
  // ...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyzRGBcloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyzCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

  // Only creates cloud for lower half of image
  for ( i = IMAGEHEIGHT/2; i < IMAGEHEIGHT; i++ )
  {
    for ( j = 0; j < IMAGEWIDTH; j++ )
    {
      unsigned short disparity = (unsigned short)biggerDisp.at<uchar>(i, j);
      // do not save invalid points
      if ( disparity>6 && disparity < 255 )
      {
        float z = (FOCALLENTH*BASELINE)/(float)disparity;
        float x = (j*z)/FOCALLENTH; //i = row
        float y = (i*z)/FOCALLENTH; //j = col

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        point_xyzCloudPtr->points.push_back(point);

        // get color values
        b = (int)biggerLOI.at<cv::Vec3b>(i,j)[0];
        g = (int)biggerLOI.at<cv::Vec3b>(i,j)[1];
        r = (int)biggerLOI.at<cv::Vec3b>(i,j)[2];

        pcl::PointXYZRGB pointRGB;
        pointRGB.x = x;
        pointRGB.y = y;
        pointRGB.z = z;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        pointRGB.rgb = *reinterpret_cast<float*>(&rgb);
        point_xyzRGBcloud_ptr->points.push_back(pointRGB);
      }
    }
  }

  point_xyzRGBcloud_ptr->width = (int) point_xyzRGBcloud_ptr->points.size();
  point_xyzRGBcloud_ptr->height = 1;
  point_xyzRGBcloud_ptr->is_dense = false;
  //pcl::io::savePLYFileASCII("point_xyzRGBcloud_ptr.ply",*point_xyzRGBcloud_ptr);

  point_xyzCloudPtr->width = (int) point_xyzCloudPtr->points.size();
  point_xyzCloudPtr->height = 1;
  point_xyzCloudPtr->is_dense = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyzRGBcloudFiltered_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  

/*
  // Filter objects futher than 10 meters away
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  
  pass.setInputCloud (point_xyzRGBcloud_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0,20.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*point_xyzRGBcloudFiltered_ptr);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0,20.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*point_xyzRGBcloudFiltered_ptr);
  pcl::io::savePLYFileASCII("output/point_xyzRGBcloud_ptrTOOT.ply",*point_xyzRGBcloudFiltered_ptr);

  */
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (point_xyzRGBcloud_ptr);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*point_xyzRGBcloudFiltered_ptr);

 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> inliers;
  std::vector<int> coefInliers;
  Eigen::VectorXf model_coefficients;
  int sampleIterations = 10;
  // created RandomSampleConsensus object and compute the appropriated model
  
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (point_xyzRGBcloudFiltered_ptr));
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
  ransac.setDistanceThreshold (0.2);
  ransac.computeModel();
  ransac.getInliers(inliers);

  model_p->getSamples(sampleIterations, coefInliers);
  //coefInliers.push_back(inliers[0]);
  //coefInliers.push_back(inliers[sizeof(inliers)/2]);
  //coefInliers.push_back(inliers[sizeof(inliers)]);
  model_p->computeModelCoefficients(coefInliers, model_coefficients);
  //std::cout << "Coefficients: " << model_coefficients <<  std::endl;
  model_p->optimizeModelCoefficients (inliers, model_coefficients, opt_model_coefficients);
  //std::cout << "opt_model_coefficients: " << opt_model_coefficients <<  std::endl;

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZRGB>(*point_xyzRGBcloudFiltered_ptr, inliers, *final);

  //pcl::io::savePLYFileASCII("road.ply",*final);
  
  //std::cout << "PointCloud before filtering has: " << point_xyzRGBcloudFiltered_ptr->points.size() << " data points." << std::endl;
  return true;
}

bool PointCloud::projectRegionToPointCloud(cv::Rect regionRect)
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

  // Only creates cloud for found rect
  for ( i = regionRect.y; i < regionRect.y+regionRect.height; i++ )
  {
    for ( j = regionRect.x; j < regionRect.x+regionRect.width; j++ )
    {
      unsigned short disparity = (unsigned short)biggerDisp.at<uchar>(i, j);

      // do not save invalid points
      if ( disparity>1 && disparity < 255 )
      {
        float z = (FOCALLENTH*BASELINE)/(float)disparity;
        float x = (j*z)/FOCALLENTH; //i = row
        float y = (i*z)/FOCALLENTH; //j = col

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        point_xyzCloudPtr->points.push_back(point);

        // get color values
        b = (int)biggerLOI.at<cv::Vec3b>(i,j)[0];
        g = (int)biggerLOI.at<cv::Vec3b>(i,j)[1];
        r = (int)biggerLOI.at<cv::Vec3b>(i,j)[2];

        pcl::PointXYZRGB pointRGB;
        pointRGB.x = x;
        pointRGB.y = y;
        pointRGB.z = z;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        pointRGB.rgb = *reinterpret_cast<float*>(&rgb);
        point_xyzRGBcloud_ptr->points.push_back(pointRGB);
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
  //std::cout << "PointCloud before filtering has: " << point_xyzCloudPtr->points.size() << " data points." << std::endl;
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
    cluster3DWidthHeightDepth.x = (point_xyzCloudPtr->points[indexMaxX].x - point_xyzCloudPtr->points[indexMinX].x);
    cluster3DWidthHeightDepth.y = (point_xyzCloudPtr->points[indexMaxY].y - point_xyzCloudPtr->points[indexMinY].y);
    cluster3DWidthHeightDepth.z = (point_xyzCloudPtr->points[indexMaxZ].z - point_xyzCloudPtr->points[indexMinZ].z);
    //cout << "x: " << cluster3DWidthHeightDepth.x << " y: " << cluster3DWidthHeightDepth.y << " z: " << cluster3DWidthHeightDepth.z << endl;
    

    clusterFront3Dpoint.x = point_xyzCloudPtr->points[indexMinZ].x;
    clusterFront3Dpoint.y = point_xyzCloudPtr->points[indexMinZ].y;
    clusterFront3Dpoint.z = point_xyzCloudPtr->points[indexMinZ].z;

    // Find point to plane distance 
    pointDistanceFromPlane = pcl::pointToPlaneDistanceSigned(clusterFront3Dpoint, opt_model_coefficients);
    
    // project point onto plane 
    pcl::projectPoint(clusterFront3Dpoint, opt_model_coefficients, pointProjected2Plane); 
/*
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
     
    std::stringstream ss;
    ss << "output/clusters/cloud_" << vIt << "_distance_" << clusterCenter3Dpoint.z << ".ply";
    pcl::io::savePLYFileASCII(ss.str (),*point_xyzRGBcloud_ptr); 
    vIt++;
    */
    return true;
  }
  return false;
}

bool PointCloud::createCompletePointCloud(cv::Mat RGBcolorImage)
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

  // Only creates cloud for lower half of image
  for ( i = 0; i < IMAGEHEIGHT; i++ )
  {
    for ( j = 0; j < IMAGEWIDTH; j++ )
    {
      unsigned short disparity = (unsigned short)biggerDisp.at<uchar>(i, j);
      // do not save invalid points
      if ( disparity>1 && disparity < 255 )
      {
        float z = (FOCALLENTH*BASELINE)/(float)disparity;
        float x = (j*z)/FOCALLENTH; //i = row
        float y = (i*z)/FOCALLENTH; //j = col

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        point_xyzCloudPtr->points.push_back(point);

        // get color values
        b = (int)RGBcolorImage.at<cv::Vec3b>(i,j)[0];
        g = (int)RGBcolorImage.at<cv::Vec3b>(i,j)[1];
        r = (int)RGBcolorImage.at<cv::Vec3b>(i,j)[2];

        pcl::PointXYZRGB pointRGB;
        pointRGB.x = x;
        pointRGB.y = y;
        pointRGB.z = z;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        pointRGB.rgb = *reinterpret_cast<float*>(&rgb);
        point_xyzRGBcloud_ptr->points.push_back(pointRGB);
      }
    }
  }

  point_xyzRGBcloud_ptr->width = (int) point_xyzRGBcloud_ptr->points.size();
  point_xyzRGBcloud_ptr->height = 1;
  point_xyzRGBcloud_ptr->is_dense = false;

  point_xyzCloudPtr->width = (int) point_xyzCloudPtr->points.size();
  point_xyzCloudPtr->height = 1;
  point_xyzCloudPtr->is_dense = false;

  //pcl::io::savePLYFileASCII("output/point_xyzRGBcloud_ptrComplete.ply",*point_xyzRGBcloud_ptr);

  return true;
}

cv::Point3f PointCloud::projectFrom3Dto2D(cv::Point3f world3Dcoordinate)
{
    cv::Point3f world2Dcoordiante;
    world2Dcoordiante.x = (world3Dcoordinate.x*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.y = (world3Dcoordinate.y*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.z = (world3Dcoordinate.x*FOCALLENTH)/world2Dcoordiante.x;
    return world2Dcoordiante;
}



