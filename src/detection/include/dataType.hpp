#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <sensor_msgs/PointCloud2.h>
 
#include "detection/Centroids.h"
#include "detection/CloudCluster.h"
#include "detection/CloudClusterArray.h"
#include "detection/DetectedObject.h"
#include "detection/DetectedObjectArray.h"

#define __APP_NAME__ "euclidean_clustering"