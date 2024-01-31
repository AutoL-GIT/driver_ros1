
#include <ros/ros.h>
#include <autol_msgs/AutolPacket.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace example
{
    class RcvPcd
    {
    public:
        RcvPcd(ros::NodeHandle nh, ros::NodeHandle private_nh);
        ~RcvPcd(){}

        void DoSomthing();
        void pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcd);
    private:
    };
}
