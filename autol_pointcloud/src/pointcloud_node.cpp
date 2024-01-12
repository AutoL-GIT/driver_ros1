#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include "autol_pointcloud/convert.h"

void cloud_cb (const autol_msgs::AutolFrame& frameMsg)
{
    ROS_INFO("cloud_cb"); 
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    autol_pointcloud::Convert convert(nh, private_nh);

    return 0;
}
