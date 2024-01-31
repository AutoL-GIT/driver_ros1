#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include "example_1/RcvPcd.h"

using namespace example;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RcvPcd rcv_pcd =  RcvPcd(nh, private_nh);

  rcv_pcd.DoSomthing();
  
  return 0;
}

