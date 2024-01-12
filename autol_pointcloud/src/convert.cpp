#include "autol_pointcloud/convert.h"

namespace autol_pointcloud
{
    Convert::Convert(ros::NodeHandle nh, ros::NodeHandle private_nh):
    raw_data_(new autol_data::RawData())
    {
        ROS_INFO("Convert()"); 

        private_nh.param("slam", is_slam_active_, false);

        raw_data_->Setup();

        convert_data_ = boost::shared_ptr<autol_data::PointcloudXYZ>(new autol_data::PointcloudXYZ(0, 0, "map", "map", raw_data_->pointsPerPacket()));

        pub_autol_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2> ("autol_pointcloud", 1);
        ros::Subscriber sub_frame = nh.subscribe ("autol_frame_data", 10, &Convert::frameData_cb, (Convert *)this,
            ros::TransportHints().tcpNoDelay(true));

        ros::spin ();
        
        is_previous_frame_processing_ = false;
    }

    void Convert::frameData_cb (const autol_msgs::AutolFrame::ConstPtr &frameMsg)
    {    
        if(is_previous_frame_processing_ == true)
        {
            ROS_INFO("Previous frame processing"); 
            return;
        }
        is_previous_frame_processing_ = true;
    
    	lidar_data_update_map_[frameMsg->lidar_index] = true;
    	for(int i = 0; i< frameMsg->packets.size(); ++i)
    	    lidar_data_map_[frameMsg->lidar_index].push_back(frameMsg->packets[i]);
    	
        bool is_all_update = true;
        
        for(auto& lidar : lidar_data_update_map_)
        {
            if(lidar.second == false)
            {
                is_all_update = false;
        	 break;
            }
        }
        
        if(is_all_update)
        {
           autol_msgs::AutolFramePtr merged_frame(new autol_msgs::AutolFrame);
        	
            int packet_size = 0;
            for(auto& lidar : lidar_data_map_)
            {
                packet_size += lidar.second.size();
            }
        
            merged_frame->packets.reserve(packet_size);
        
            int point_accum_count = 0;
	    for(auto& lidar : lidar_data_map_)
            {
                merged_frame->packets.insert(merged_frame->packets.end(), lidar.second.begin(), lidar.second.end());
                point_accum_count += lidar.second.size();
		point_count_by_lidar_map_[lidar.first] = point_accum_count;
            }

            for(auto& lidar : lidar_data_update_map_)
                lidar.second = false;
 	    for(auto& lidar : lidar_data_map_)
                lidar.second.clear();
            
            convert_data_->Setup(merged_frame);
            for (size_t i = 0; i < merged_frame->packets.size(); ++i)
            {
            	int lidar_index_of_point = 0;
            	std::map<int, int>::reverse_iterator ritr;
            	for(ritr=point_count_by_lidar_map_.rbegin(); ritr != point_count_by_lidar_map_.rend(); ++ritr)
            	{
         	    if(i < ritr->second)
                        lidar_index_of_point = ritr->first;
            	}

                raw_data_->ConvertFromRaw(merged_frame->packets[i], *convert_data_, is_slam_active_, lidar_index_of_point);
            }
            pub_autol_pointcloud_.publish(convert_data_->resizeData());
        }
        else
        {}
        
        is_previous_frame_processing_ = false;

    }
}
