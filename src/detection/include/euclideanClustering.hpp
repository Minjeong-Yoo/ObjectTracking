
#ifndef __EUCLIDEANCLUSTERING_HPP__
#define __EUCLIDEANCLUSTERING_HPP__

#include "dataType.hpp"
// S#include "rayGroundFilter.hpp"

class EuclideanClustering
{
public:
    // Constructor
    explicit EuclideanClustering();
    virtual ~EuclideanClustering();

private:
    // Node Handler
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber _sub_velodyne;

    // Publisher
    ros::Publisher _pub_cluster_cloud;
    ros::Publisher _pub_ground_cloud;
    ros::Publisher _centroid_pub;
    ros::Publisher _pub_clusters_message;
    ros::Publisher _pub_noground_cloud;
    ros::Publisher _pub_detected_objects;

    // Check Publisher
    ros::Publisher _pub_removePointsUpTo;
    ros::Publisher _pub_downsampleCloud;
    ros::Publisher _pub_clipCloud;
    ros::Publisher _pub_keepLanePoints;
    ros::Publisher _pub_toRayGroundFilter; 

    // velodyne header 
    std_msgs::Header _velodyne_header;

    // Param
    bool _remove_ground_ransac;
    bool _remove_ground_rayGroundFilter;
    bool _downsample_cloud;
    double _leaf_size;
    int _cluster_size_min;
    int _cluster_size_max;
    bool _use_diffnormals;
    bool _pose_estimation;
    bool _keep_lanes;
    double _keep_lane_left_distance;
    double _keep_lane_right_distance;
    bool _clipCloud;
    double _clip_min_height;
    double _clip_max_height;
    double _remove_points_upto;
    double _clustering_distance;
    double _cluster_merge_threshold;
    bool _use_gpu;
    bool _use_multiple_thres;
    bool _output_frame;
    float _in_max_height;
    float _in_floor_max_angle;

    // Ray Ground Param
    double _general_max_slope;
    double _local_max_slope;
    double _radial_divider_angle;
    double _concentric_divider_distance;
    double _min_height_threshold;
    double _clipping_height;
    double _min_point_distance;
    double _reclass_distance_threshold;
    
  
    std::string str_distances;
    std::string str_ranges;

    // PointCloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr;

    // RayGroundFilter class object 
    // RayGroundFilter rayGroundFilter;
    // const sensor_msgs::PointCloud2::ConstPtr filtered_cloud_ptr;
    
private: 
    // Common function
    void getParam();

    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud);
    void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr);

    // Pre-processing function
    bool removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);
    bool downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2);
    bool clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height = -1.3, float in_max_height = 0.5);
    bool keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5, float in_right_lane_threshold = 1.5);
    bool removeFloor_ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height = 0.2, float in_floor_max_angle = 0.1);

    // Segmentation function 
    




};

#endif