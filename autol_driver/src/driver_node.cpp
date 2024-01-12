#include "ros/ros.h"                            // ROS Default Header File

// #include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <signal.h>
#include "autol_driver/driver.h"


autol_driver::AutolDriver* dvr = NULL;

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloudsrc)
{
  sensor_msgs::PointCloud2 cloudmsg;
  pcl::toROSMsg(cloudsrc, cloudmsg);
  cloudmsg.header.frame_id = "map";
  return cloudmsg;
}


void SigintHandler(int sig)
{
  dvr->Dispose();

  if(dvr != NULL)
    delete(dvr);

  ros::shutdown();
}

int main(int argc, char **argv)           // Node Main Function
{
  ros::init(argc, argv, "driver_node");   // Initializes Node Name
  ros::NodeHandle nh;                     // Node handle declaration for communication with ROS system
  ros::NodeHandle private_nh("~");        // Node handle declaration for communication with ROS system

  signal(SIGINT, SigintHandler);

  dvr = new autol_driver::AutolDriver(nh, private_nh);
  dvr->StartRecvPacket();

  ROS_INFO("closing"); 
  return 0;
}

