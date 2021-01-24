#include "euclideanClustering.hpp"

EuclideanClustering::EuclideanClustering() 
: current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
  downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
  nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
  clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)     // PointCloud initializatio
{
    // Publisher
    _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
    _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
    _centroid_pub = nh.advertise<detection::Centroids>("/cluster_centroids",1);

    _pub_noground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_nogroud",1);
    _pub_clusters_message = nh.advertise<detection::CloudClusterArray>("/cloud_clusters",1);
    _pub_detected_objects = nh.advertise<detection::DetectedObjectArray>("/objects",1);

    // Check Publisher
    _pub_removePointsUpTo = nh.advertise<sensor_msgs::PointCloud2>("/removePointsUpTo", 1);
    _pub_downsampleCloud = nh.advertise<sensor_msgs::PointCloud2>("/downsampleCloud", 1);
    _pub_clipCloud = nh.advertise<sensor_msgs::PointCloud2>("/clipCloud",1);
    _pub_keepLanePoints = nh.advertise<sensor_msgs::PointCloud2>("/keepLanePoints", 1);
    _pub_toRayGroundFilter = nh.advertise<sensor_msgs::PointCloud2>("/toRayGroundFilter", 1);

    // Subscriber
   _sub_velodyne = nh.subscribe("/velodyne_points", 1, &EuclideanClustering::velodyneCallback, this);

    // getParam();
    getParam();
//     rayGroundFilter.initParam(
//         _general_max_slope,
//         _local_max_slope,
//         _radial_divider_angle,
//         _concentric_divider_distance,
//         _min_height_threshold,
//         _clipping_height,
//         _min_point_distance,
//         _reclass_distance_threshold
//     );
}

EuclideanClustering::~EuclideanClustering()
{

}

void EuclideanClustering::getParam()
{
    
    nh.getParam("euclideanClustering/downsample_cloud", _downsample_cloud);
    nh.getParam("euclideanClustering/remove_ground_ransac", _remove_ground_ransac);
    nh.getParam("euclideanClustering/remove_ground_rayGroundFilter", _remove_ground_rayGroundFilter);
    nh.getParam("euclideanClustering/leaf_size", _leaf_size);
    nh.getParam("euclideanClustering/cluster_size_min", _cluster_size_min);
    nh.getParam("euclideanClustering/cluster_size_max", _cluster_size_max);
    nh.getParam("euclideanClustering/pose_estimation", _pose_estimation);
    nh.getParam("euclideanClustering/clipCloud", _clipCloud);
    nh.getParam("euclideanClustering/clip_min_height", _clip_min_height);
    nh.getParam("euclideanClustering/clip_max_height", _clip_max_height);
    nh.getParam("euclideanClustering/keep_lanes", _keep_lanes);
    nh.getParam("euclideanClustering/keep_lane_left_distance", _keep_lane_left_distance);
    nh.getParam("euclideanClustering/keep_lane_right_distance", _keep_lane_right_distance);
    nh.getParam("euclideanClustering/cluster_merge_threshold", _cluster_merge_threshold);
    nh.getParam("euclideanClustering/output_frame", _output_frame);
    nh.getParam("euclideanClustering/remove_points_upto", _remove_points_upto);
    nh.getParam("euclideanClustering/clustering_distance", _clustering_distance);
    nh.getParam("euclideanClustering/use_gpu", _use_gpu);
    nh.getParam("euclideanClustering/use_multiple_thres", _use_multiple_thres);
    nh.getParam("euclideanClustering/clustering_distances", str_distances);
    nh.getParam("euclideanClustering/clustering_ranges", str_ranges);
    nh.getParam("euclideanClustering/in_max_height", _in_max_height);
    nh.getParam("euclideanClustering/in_floor_max_angle", _in_floor_max_angle);

    // Ray Ground Get Param
    // nh.getParam("euclideanClustering/general_max_slope", _general_max_slope);
    // nh.getParam("euclideanClustering/local_max_slope", _local_max_slope);
    // nh.getParam("euclideanClustering/radial_divider_angle", _radial_divider_angle);
    // nh.getParam("euclideanClustering/concentric_divider_distance", _concentric_divider_distance);
    // nh.getParam("euclideanClustering/min_height_threshold", _min_height_threshold);
    // nh.getParam("euclideanClustering/clipping_height", _clipping_height);
    // nh.getParam("euclideanClustering/min_point_distance", _min_point_distance);
    // nh.getParam("euclideanClustering/reclass_distance_threshold", _reclass_distance_threshold);
}

void EuclideanClustering::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud)
{
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    _velodyne_header = in_sensor_cloud->header;

    // ego vehicle 주변 ROI 선정
    if (_remove_points_upto > 0.0)
    {
        if (!removePointsUpTo(current_sensor_cloud_ptr, removed_points_cloud_ptr, _remove_points_upto)) ROS_ERROR_STREAM("Fail Convert Message to PointCloud");
        publishCloud(&_pub_removePointsUpTo, removed_points_cloud_ptr);

    }
    else removed_points_cloud_ptr = current_sensor_cloud_ptr;

    // Downsampling
    if (_downsample_cloud)
    {
        if (!downsampleCloud(removed_points_cloud_ptr, downsampled_cloud_ptr, _leaf_size)) ROS_ERROR_STREAM("Fail to downsampling");
        publishCloud(&_pub_downsampleCloud, downsampled_cloud_ptr);

    }
    else downsampled_cloud_ptr = removed_points_cloud_ptr; 
    
    // Trim with z axis
    if (_clipCloud)
    {
       if (!clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height)) ROS_ERROR_STREAM("Fail to clip"); 
       publishCloud(&_pub_clipCloud, clipped_cloud_ptr);    
    }
    else clipped_cloud_ptr = downsampled_cloud_ptr;

    if (_keep_lanes)
    {
        if(!keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance)) ROS_ERROR_STREAM("Fail to keepLanePoints");
        publishCloud(&_pub_keepLanePoints, inlanes_cloud_ptr);

    }
    else inlanes_cloud_ptr = clipped_cloud_ptr;
    
    
    if (_remove_ground_ransac)
    {
        if(!removeFloor_ransac(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr, _in_max_height, _in_floor_max_angle)) ROS_ERROR_STREAM("Fail to remove floor");
        publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
    }
    // else if (_remove_ground_rayGroundFilter)
    // {
    //     // rayGroundFilter input : sensor_msgs::Pointcloud2 
    //     // 형변환
    //     sensor_msgs::PointCloud2 cloud_msg;
    //     sensor_msgs::PointCloud2 output_msg;
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //     pcl::toROSMsg(*inlanes_cloud_ptr, cloud_msg);
    //     cloud_msg.header = _velodyne_header;
    //     sensor_msgs::PointCloud2::ConstPtr cloud_msg_ptr(new sensor_msgs::PointCloud2(cloud_msg));

    //     // check
    //     // RayGroundFilter 호출
    //     if(!rayGroundFilter.removeFloor_rayGroundFilter(cloud_msg_ptr, filtered_cloud_ptr)) ROS_ERROR_STREAM("Fail to remove floor with Rayground filter");
    //     // output_msg = *filtered_cloud_ptr;
    //     // _pub_toRayGroundFilter.publish(output_msg);
    //     pcl::fromROSMsg(*filtered_cloud_ptr, *output_cloud);
    //     publishCloud(&_pub_toRayGroundFilter, output_cloud);        
        
    // }
    else nofloor_cloud_ptr = inlanes_cloud_ptr;
    publishCloud(&_pub_noground_cloud, nofloor_cloud_ptr);
    

}

// // Ray ground filter 입력 확인 
// bool EuclideanClustering::removeFloor_rayGroundFilter(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud)
// {
//     ROS_WARN_STREAM("inside");
//     _pub_toRayGroundFilter.publish(in_sensor_cloud);
//     return true;
// }

bool EuclideanClustering::removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
    if (origin_distance > in_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
  return true;
}

bool EuclideanClustering::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  bool _remove_ground;
  sor.setInputCloud(in_cloud_ptr);
  sor.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);
  sor.filter(*out_cloud_ptr);

  return true;
}

bool EuclideanClustering::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                    float in_min_height, float in_max_height)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].z >= in_min_height && in_cloud_ptr->points[i].z <= in_max_height)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
  return true;
}

bool EuclideanClustering:: keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                          float in_left_lane_threshold, float in_right_lane_threshold)
{
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
    {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
  
  return true;
}

void EuclideanClustering::publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);
}

bool EuclideanClustering::removeFloor_ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, 
                                      float in_max_height,
                                      float in_floor_max_angle)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);

  return true;
}         

