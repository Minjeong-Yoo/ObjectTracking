#include "rayGroundFilter.hpp"

// Constructor
RayGroundFilter::RayGroundFilter()
{
  
}

// Destructor
RayGroundFilter::~RayGroundFilter()
{

}

// Get param 
void RayGroundFilter::initParam(
    double general_max_slope,
    double local_max_slope,
    double radial_divider_angle,
    double concentric_divider_distance,
    double min_height_threshold,
    double clipping_height,
    double min_point_distance,
    double reclass_distance_threshold
)
{
    _general_max_slope = general_max_slope;
    _local_max_slope = local_max_slope;
    _radial_divider_angle = radial_divider_angle;
    _concentric_divider_distance = concentric_divider_distance;
    _min_height_threshold = min_height_threshold;
    _clipping_height = clipping_height;
    _min_point_distance = min_point_distance;
    _reclass_distance_threshold = reclass_distance_threshold;

}

bool RayGroundFilter::removeFloor_rayGroundFilter(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud, const sensor_msgs::PointCloud2::ConstPtr& out_sensor_cloud)
{
    std::vector<PointCloudRH> radial_ordered_clouds;
    std::vector<void*> ground_ptrs, no_ground_ptrs;

    _radial_dividers_num = ceil(360.0 / _radial_divider_angle);
    const size_t point_count = in_sensor_cloud->width*in_sensor_cloud->height;

    sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);     // constPtr -> Ptr 변환 

    if(!ConvertAndTrim(in_sensor_cloud, trans_sensor_cloud, _clipping_height, _min_point_distance, &radial_ordered_clouds, &no_ground_ptrs)) ROS_ERROR_STREAM("Fail to convert and trim");
    else if(!ClassifyPointCloud(radial_ordered_clouds, point_count, &ground_ptrs, &no_ground_ptrs)) ROS_ERROR_STREAM("Fail to classify pointcloud");
    else if(!FilteredCloud(in_sensor_cloud, ground_ptrs, out_sensor_cloud)) ROS_ERROR_STREAM("Fail to filtering cloud");

    return true;
}

bool RayGroundFilter::ConvertAndTrim(const sensor_msgs::PointCloud2::ConstPtr in_cloud_ptr,
                                    const sensor_msgs::PointCloud2::Ptr& out_cloud_ptr,
                                    const double in_clip_height,
                                    double in_min_distance,
                                    std::vector<PointCloudRH>* out_radial_ordered_clouds,
                                    std::vector<void*>* out_no_ground_ptrs)
{
  *out_cloud_ptr = *in_cloud_ptr; 

  // --- Clarify some of the values used to access the binary blob
  size_t point_size = out_cloud_ptr->row_step/out_cloud_ptr->width;  // in Byte   // 한 포인트 당 byte 개수 
  size_t cloud_count = out_cloud_ptr->width*out_cloud_ptr->height;

  const uint offset_not_set = ~0;
  uint x_offset = offset_not_set;  // in Byte from the point's start
  uint y_offset = offset_not_set;  // in Byte from the point's start
  uint z_offset = offset_not_set;  // in Byte from the point's start

  if (out_cloud_ptr->fields.size() < 3)
  {
    ROS_ERROR_STREAM_THROTTLE(10, "Failed to decode the pointcloud message : not enough fields found : "
        << out_cloud_ptr->fields.size() << " (needs at least 3 : x,y,z)");
    return false;
  }

  for ( uint i = 0; i < out_cloud_ptr->fields.size(); i++ )
  {
    sensor_msgs::PointField field = out_cloud_ptr->fields[i];
    if ("x" == field.name)
    {
      x_offset = field.offset;      // 0
    }
    else if ("y" == field.name)
    {
      y_offset = field.offset;      // 4
    }
    else if ("z" == field.name)
    {
      z_offset = field.offset;      // 8
    }
  }

  if (offset_not_set == x_offset || offset_not_set == y_offset || offset_not_set == z_offset)
  {
    ROS_ERROR_STREAM_THROTTLE(10, "Failed to decode the pointcloud message : bad coordinate field name");
    return false;
  }
 ROS_WARN_STREAM("test8");

    ROS_INFO_STREAM(_radial_dividers_num);
  out_radial_ordered_clouds->resize(_radial_dividers_num);

  const int mean_ray_count = cloud_count/_radial_dividers_num;      // Each radial divider마다 cloud 평균 개수 
  // In theory reserving more than the average memory would reduce even more the number of realloc
  // but it would also make the reserving takes longer. One or two times the average are pretty
  // much identical in term of speedup. Three seems a bit worse.
  const int reserve_count = mean_ray_count;
  for (auto it = out_radial_ordered_clouds->begin(); it != out_radial_ordered_clouds->end(); it++)
  {
    it->reserve(reserve_count);             // each radial divider 마다 cloud 평균 개수 크기 만큼 할당 
  }
  ROS_INFO_STREAM(cloud_count);

  for ( size_t i = 0; i < cloud_count; i++ )
  { 
      ROS_WARN_STREAM("test14");
    // --- access the binary blob fields
    uint8_t* point_start_ptr = reinterpret_cast<uint8_t*>(out_cloud_ptr->data.data()) + (i*point_size);
    float x = *(reinterpret_cast<float*>(point_start_ptr+x_offset));
    float y = *(reinterpret_cast<float*>(point_start_ptr+y_offset));
    float z = *(reinterpret_cast<float*>(point_start_ptr+z_offset));
    if (is_big_endian() != out_cloud_ptr->is_bigendian)
    {
      x = ReverseFloat(x);
      y = ReverseFloat(y);
      z = ReverseFloat(z);
    }
    ROS_WARN_STREAM("test14");
    // ---

    if (z > in_clip_height)
    {
      out_no_ground_ptrs->push_back(point_start_ptr);
      continue;
    }
    auto radius = static_cast<float>(sqrt(x*x + y*y));
    if (radius < in_min_distance)
    {
      out_no_ground_ptrs->push_back(point_start_ptr);
      continue;
    }
ROS_WARN_STREAM("test14");
#ifdef USE_ATAN_APPROXIMATION

    auto theta = static_cast<float>(fast_atan2(y, x) * 180 / M_PI);
#else
    ROS_WARN_STREAM("test14");
    auto theta = static_cast<float>(atan2(y, x) * 180 / M_PI);
#endif  // USE_ATAN_APPROXIMATION
   
    if (theta < 0)
    {
      theta += 360;
    }
    else if (theta >= 360)
    {
      theta -= 360;
    }

    // radial_divider_angle_ is computed so that
    // 360 / radial_divider_angle_ = radial_dividers_num_
    // Even though theta < 360, rounding error may make it so that
    // theta / radial_divider_angle_ >= radial_dividers_num_
    // which gives a radial_div one past the end. The modulo is here to fix
    // this rare case, wrapping the bad radial_div back to the first one.
    auto radial_div = (size_t)floor(theta / _radial_divider_angle) % _radial_dividers_num;
    out_radial_ordered_clouds->at(radial_div).emplace_back(z, radius, point_start_ptr);
     ROS_WARN_STREAM("test");
  }  // end for

  // order radial points on each division
    auto strick_weak_radius_ordering = [](const PointRH& a, const PointRH& b)
    {
      if (a.radius < b.radius)
    {
      return true;
    }
    if (a.radius > b.radius)
    {
      return false;
    }
    // then the radius are equals. We add a secondary condition to keep the sort stable
    return a.original_data_pointer < b.original_data_pointer;
    };
    for (size_t i = 0; i < _radial_dividers_num; i++)
    {
        std::sort(out_radial_ordered_clouds->at(i).begin(),
                  out_radial_ordered_clouds->at(i).end(),
                  strick_weak_radius_ordering);
  }
   
  return true;
}

bool RayGroundFilter::ClassifyPointCloud(const std::vector<PointCloudRH>& in_radial_ordered_clouds,
                        const size_t in_point_count,
                        std::vector<void*>* out_ground_ptrs,
                        std::vector<void*>* out_no_ground_ptrs)

{

  double expected_ground_no_ground_ratio = 0.1;
  out_ground_ptrs->reserve(in_point_count * expected_ground_no_ground_ratio);
  out_no_ground_ptrs->reserve(in_point_count);

  const float local_slope_ratio = tan(DEG2RAD(_local_max_slope));
  const float general_slope_ratio = tan(DEG2RAD(_general_max_slope));
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++)  // sweep through each radial division
  {
    float prev_radius = 0.f;
    float prev_height = 0.f;
    bool prev_ground = false;
    bool current_ground = false;
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++)  // loop through each point in the radial div
    {
      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;          // ordered된 이전, 현재 point 사이의 radius 거리차
      float height_threshold = local_slope_ratio * points_distance;                         // tan 공식 => y = tan(theta) * x   // tan(theta)=> local_slope_ratio 
      float current_height = in_radial_ordered_clouds[i][j].height;
      float general_height_threshold = general_slope_ratio * in_radial_ordered_clouds[i][j].radius;

      // for points which are very close causing the height threshold to be tiny, set a minimum value
      if (points_distance > _concentric_divider_distance && height_threshold < _min_height_threshold)
      {
        height_threshold = _min_height_threshold;
      }

      // check current point height against the LOCAL threshold (previous point)
      if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
      {
        // Check again using general geometry (radius from origin) if previous points wasn't ground
        if (!prev_ground)
        {
          if (current_height <= general_height_threshold && current_height >= -general_height_threshold)
          {
            current_ground = true;
          }
          else
          {
            current_ground = false;
          }
        }
        else
        {
          current_ground = true;
        }
      }
      else
      {
        // check if previous point is too far from previous one, if so classify again
        if (points_distance > _reclass_distance_threshold &&
            (current_height <= height_threshold && current_height >= -height_threshold))
        {
          current_ground = true;
        }
        else
        {
          current_ground = false;
        }
      }

      if (current_ground)
      {
        out_ground_ptrs->push_back(in_radial_ordered_clouds[i][j].original_data_pointer);
        prev_ground = true;
      }
      else
      {
        out_no_ground_ptrs->push_back(in_radial_ordered_clouds[i][j].original_data_pointer);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].height;
    }
  }
}

bool RayGroundFilter::FilteredCloud(const sensor_msgs::PointCloud2::ConstPtr in_origin_cloud, const std::vector<void*>& in_selector, sensor_msgs::PointCloud2::ConstPtr out_cloud_msg)
{
        ROS_WARN_STREAM("test3");

  sensor_msgs::PointCloud2::Ptr out_filtered_msg;  
  size_t point_size = in_origin_cloud->row_step/in_origin_cloud->width;  // in Byte

  // TODO(yoan picchi) I fear this may do a lot of cache miss because it is sorted in the radius
  // and no longer sorted in the original pointer. One thing to try is that, given
  // that we know the value possibles, we can make a rather large vector and insert
  // all the point in, then move things around to remove the "blank" space. This
  // would be a linear sort to allow cache prediction to work better. To be tested.

  size_t data_size = point_size * in_selector.size();
  out_filtered_msg->data.resize(data_size);  // TODO(yoan picchi) a fair amount of time (5-10%) is wasted on this resize

  size_t offset = 0;
  for ( auto it = in_selector.cbegin(); it != in_selector.cend(); it++ )
  {
    memcpy(out_filtered_msg->data.data()+offset, *it, point_size);
    offset += point_size;
  }

  out_filtered_msg->width  = (uint32_t) in_selector.size();
  out_filtered_msg->height = 1;
  
  out_filtered_msg->fields            = in_origin_cloud->fields;
  out_filtered_msg->header.frame_id   = in_origin_cloud->header.frame_id;
  out_filtered_msg->header.stamp      = in_origin_cloud->header.stamp;
  out_filtered_msg->point_step        = in_origin_cloud->point_step;
  out_filtered_msg->row_step          = point_size * in_selector.size();
  out_filtered_msg->is_dense          = in_origin_cloud->is_dense
                                        && in_origin_cloud->data.size() == in_selector.size();

  out_cloud_msg = out_filtered_msg;

}



bool RayGroundFilter::is_big_endian(void)
{
  union
  {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 1;
}

float RayGroundFilter::ReverseFloat(float inFloat)  // Swap endianness
{
  float retVal;
  char *floatToConvert = reinterpret_cast<char*>(& inFloat);
  char *returnFloat = reinterpret_cast<char*>(& retVal);

  // swap the bytes into a temporary buffer
  returnFloat[0] = floatToConvert[3];
  returnFloat[1] = floatToConvert[2];
  returnFloat[2] = floatToConvert[1];
  returnFloat[3] = floatToConvert[0];

  return retVal;
}

float RayGroundFilter::fast_atan2(float y, float x)
{
  constexpr float scaling_constant = 0.28086f;

  if (x == 0.0f) {
    // Special case atan2(0.0, 0.0) = 0.0
    if (y == 0.0f) {
      return 0.0f;
    }

    // x is zero so we are either at pi/2 for (y > 0) or -pi/2 for (y < 0)
    return ::std::copysign(PI_2, y);
  }

  // Calculate quotient of y and x
  float div = y / x;

  // Determine in which octants we can be, if |y| is smaller than |x| (|div|<1)
  // then we are either in 1,4,5 or 8 else we are in 2,3,6 or 7.
  if (fabsf(div) < 1.0f) {
    // We are in 1,4,5 or 8

    float atan = div / (1.0f + scaling_constant * div * div);

    // If we are in 4 or 5 we need to add pi or -pi respectively
    if (x < 0.0f) {
      return ::std::copysign(PI, y) + atan;
    }
    return atan;
  }

  // We are in 2,3,6 or 7
  return ::std::copysign(PI_2, y) - div / (div * div + scaling_constant);
}
