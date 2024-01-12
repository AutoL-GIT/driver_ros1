#pragma once
#include "autol_pointcloud/convertdatabase.h"

namespace autol_data
{
    class PointcloudXYZ : public autol_data::ConvertDataBase
    {
    public:        
    PointcloudXYZ(const double max_range, const double min_range, const std::string& target_frame,
                  const std::string& fixed_frame, const unsigned int scans_per_block,
                  boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());
    virtual void Setup(const autol_msgs::AutolFrame::ConstPtr& frame_msg);
    virtual void insertPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance, const float intensity, const float time);

    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_time;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;
    };
}  
