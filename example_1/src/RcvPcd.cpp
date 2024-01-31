#include "example_1//RcvPcd.h"

namespace example
{
    RcvPcd::RcvPcd(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        ros::Subscriber sub_pointcloud = nh.subscribe ("autol_pointcloud", 10, &RcvPcd::pointcloud_cb, (RcvPcd *)this,
            ros::TransportHints().tcpNoDelay(true));
        ros::spin ();
    }

    void RcvPcd::DoSomthing()
    {
    }

    void RcvPcd::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcd)
    {    
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcd, "x"),
                                                     iter_y(*pcd, "y"),
                                                     iter_z(*pcd, "z"),
                                                     iter_intensity(*pcd, "intensity");
        int cnt = 0;                                                     
        while(iter_x != iter_x.end())
        {
            ++cnt;
            if(cnt == 10000)
            {
                ROS_INFO("%f %f %f %f", *iter_x, *iter_y, *iter_z, *iter_intensity);
                //ROS_INFO("%f", *iter_intensity);
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }
    }
}
