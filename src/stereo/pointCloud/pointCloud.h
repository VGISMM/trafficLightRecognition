
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "../../defines.h"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class PointCloud {
public:
	PointCloud();
	void init(cv::Mat disp, cv::Mat colorImage);
	bool projectRegionToPointCloud(cv::Rect regionRect);
	bool findRoadSurfaceCoefficients();
	bool createCompletePointCloud(cv::Mat RGBcolorImage);
	
	cv::Mat biggerDisp;
    cv::Mat biggerLOI;
    cv::Point3f clusterCenter3Dpoint, cluster3DWidthHeightDepth, clusterFront3Dpoint, pointProjected2Plane;
    float pointDistanceFromPlane;
    // , clusterRight3Dpoint, clusterLeft3Dpoint, upperLeftCorner3Dpoint, lowerRightCorner3Dpoint;
    
private:
	int vIt = 0;
	cv::Point3f projectFrom3Dto2D(cv::Point3f world3Dcoordinate);
	Eigen::VectorXf opt_model_coefficients;
};