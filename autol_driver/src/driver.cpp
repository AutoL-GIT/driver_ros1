#include "autol_driver/driver.h"
#include <tf/transform_listener.h>

namespace autol_driver
{    
    AutolDriver::AutolDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        ROS_INFO("constructor : AutolDriver"); 
        
	for(int i = 0; i<6; ++i)
	{
	    PORT_LIST[i] = 5001 +i;
	}
        string pcap_path;

        private_nh.param("manufacture_id", config_.manufacture_id, std::string("autol"));
        private_nh.param("model_id", config_.model, std::string("G32"));
        private_nh.param("pcap_path", pcap_path, std::string(""));
        private_nh.param("input_type", input_type_, 1);
        private_nh.param("packet_per_frame", PACKET_PER_SECOND, 180);
        private_nh.param("framerate", FRAMERATE, 25);
        private_nh.param("lidarcount", LIDAR_COUNT, 1);
        
        private_nh.param("lidar1port", PORT_LIST[0], 5001);
	private_nh.param("lidar2port", PORT_LIST[1], 5002);
	private_nh.param("lidar3port", PORT_LIST[2], 5003);
	private_nh.param("lidar4port", PORT_LIST[3], 5004);
	private_nh.param("lidar5port", PORT_LIST[4], 5005);
	private_nh.param("lidar6port", PORT_LIST[5], 5006);
	
	
	if(LIDAR_COUNT < 1)
    	    LIDAR_COUNT = 1;
    	if(LIDAR_COUNT > 6)
    	    LIDAR_COUNT = 6;
	
	is_packet_init = false;
	
        pub_frame_ = nh.advertise<autol_msgs::AutolFrame>("autol_frame_data", 10);
      
        if (input_type_ == 1)   
        {   
            for(int i = 0; i< LIDAR_COUNT; ++i)
            {
		input_data_[i].reset(new autol_driver::SocketInput(private_nh, PORT_LIST[i]));
            }
        }
        else if (input_type_ == 2)
        {     
	    for(int i = 0; i< 1; ++i)
            {
                input_data_[0].reset(new autol_driver::PcapInput(private_nh, PORT_LIST[0], FRAMERATE * PACKET_PER_SECOND, pcap_path));                  
            }
        }    
        else if (input_type_ == 3)
        {  
        }
    }

    bool AutolDriver::Dispose()
    {
        return true;
    }
    
    void AutolDriver::StartRecvPacket()
    {                  
        if (input_type_ == 1)
        {	
            for(int i = 0; i< LIDAR_COUNT; ++i)
	        thread_pool[i] = std::thread(&AutolDriver::RecvPacket, this, i);
        
            for(int i = 0; i< LIDAR_COUNT; ++i)
	     thread_pool[i].join();
	}
	else if (input_type_ == 2)
	{
	    thread_pool[0] = std::thread(&AutolDriver::RecvPacket, this, 0);
	    thread_pool[0].join();
	}
	
    }
    
    void AutolDriver::RecvPacket(int lidar_index)
    {
        if (input_type_ == 3)   
        {
            return;
        }
        ROS_INFO("RecvPacket called : %d", lidar_index ); 

        UdpPacket* packet = new UdpPacket();

        autol_msgs::AutolPacket tmp_packet;
        autol_msgs::AutolFramePtr lidar_frame(new autol_msgs::AutolFrame);
        autol_msgs::AutolFramePtr lidar1_frame(new autol_msgs::AutolFrame);
        autol_msgs::AutolFramePtr lidar2_frame(new autol_msgs::AutolFrame);

        bool is_lidar1_active = false;
        bool is_lidar2_active = false;
        unsigned int lidar1_frame_count = 0;
        unsigned int lidar2_frame_count = 0;

        bool is_first_fov_lidar1 = true;
        bool is_first_fov_lidar2 = true;
        int lidar1_stage_count = 0;
        int lidar2_stage_count = 0;

        lidar_frame->packets.reserve(PACKET_PER_SECOND);
        lidar1_frame->packets.reserve(PACKET_PER_SECOND);
        lidar2_frame->packets.reserve(PACKET_PER_SECOND);
                
        unsigned long pack_id_cnt;
        unsigned long prev_packet_id = 0;

	is_packet_init = false;
            
        bool is_packet_lost = false;       
        while(ros::ok())
        {
        
            if(input_data_[lidar_index]->is_ready_input_ == false)
            {                
                break;
            }

            int ret = input_data_[lidar_index]->getPacket(&tmp_packet, PACKET_DATA_SIZE);

            if (ret != PACKET_DATA_SIZE)
            {	        
            ROS_INFO("err %d", lidar_index); 
                continue;
            }

            memcpy(packet, (char*)&(&tmp_packet)->data[0], PACKET_DATA_SIZE);

	    if(is_packet_init == false)
	    {
  	       if (packet->header_.data_type_ == 0xA5B3C2AA && packet->header_.top_bottom_side_ == 1)
	       {
		    PACKET_PER_SECOND = packet->header_.packet_id_ + 1;
		    is_packet_init = true;
	       }
	       continue;
            }

                    
            pack_id_cnt = packet->header_.packet_id_;
            
            if(pack_id_cnt != prev_packet_id +1)
            {
            	if(pack_id_cnt != 0)
            	{
            		is_packet_lost = true;
            		//ROS_INFO("packet lost"); 
            	}
            }
            prev_packet_id = pack_id_cnt;
                        
            
            if (packet->factory_ == 0x11)
            {
	        if(!(lidar1_frame->packets.size() == 0 && packet->header_.top_bottom_side_ == 1))
                {
                    is_lidar1_active = true;
                    lidar1_frame->packets.push_back(tmp_packet);
                }
            }
            else if (packet->factory_ == 0x12)
            {
	        if(!(lidar2_frame->packets.size() == 0 && packet->header_.top_bottom_side_ == 1))
	        {
                    is_lidar2_active = true;
                    lidar2_frame->packets.push_back(tmp_packet);
                }
            }
	
            if (packet->header_.data_type_ == 0xA5B3C2AA && packet->header_.packet_id_ != 0)
            {            
                if (packet->factory_ == 0x11)
                {        
    		    if(!(lidar1_frame->packets.size() == 0 && packet->header_.top_bottom_side_ == 1))
    		    {
                        ++lidar1_stage_count;
                    }                    

                    //if(is_first_fov_lidar1 == true)                    
                    //{                    
                    //    lidar1_frame.reset(new autol_msgs::AutolFrame);
                    //    lidar1_stage_count = 0;
                    //    is_first_fov_lidar1 = false;               
                    //}
                    if(lidar1_stage_count >= 2)
                    {                                        
                        if(lidar1_frame_count == 0)
                        {
                            lidar_frame->packets.insert(lidar_frame->packets.end(), lidar1_frame->packets.begin(), lidar1_frame->packets.end());
                        }
                        lidar1_frame.reset(new autol_msgs::AutolFrame);
                        lidar1_frame->packets.reserve(PACKET_PER_SECOND);

                        lidar1_stage_count = 0;
                        ++lidar1_frame_count;
                    }
                }
                else if (packet->factory_ == 0x12)
                {               
                    if(!(lidar2_frame->packets.size() == 0 && packet->header_.top_bottom_side_ == 1))
    		    {     
                        ++lidar2_stage_count;
                    }

                    //if(is_first_fov_lidar2 == true)
                    //{
                    //    lidar2_frame.reset(new autol_msgs::AutolFrame);
                    //    lidar2_stage_count = 0;
                    //    is_first_fov_lidar2 = false;               
                    //}

                    if(lidar2_stage_count >= 2)
                    {
                        if(lidar2_frame_count == 0)
                        {
                            lidar_frame->packets.insert(lidar_frame->packets.end(), lidar2_frame->packets.begin(), lidar2_frame->packets.end());
                        }
                        lidar2_frame.reset(new autol_msgs::AutolFrame);
                        lidar2_frame->packets.reserve(PACKET_PER_SECOND);

                        lidar2_stage_count = 0;
                        ++lidar2_frame_count;
                    }
                }
                
                if(lidar1_frame_count > 50 || lidar2_frame_count > 50)
                {
                    lidar1_frame_count = 0;
                    lidar2_frame_count = 0;

                    int active_lidar_count = 2;
                    if(lidar1_frame_count == 0)
                    {
                        is_lidar1_active = false;
                        --active_lidar_count;
                    }
                    if(lidar2_frame_count == 0)
                    {
                        is_lidar2_active = false;
                        --active_lidar_count;
                    }
                    lidar_frame.reset(new autol_msgs::AutolFrame);
                    lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);
                }

                if(((is_lidar1_active == true && lidar1_frame_count > 0) || is_lidar1_active == false) &&
                   ((is_lidar2_active == true && lidar2_frame_count > 0) || is_lidar2_active == false) &&
                   (is_lidar1_active == true || is_lidar2_active == true))
                {
                    int active_lidar_count = 0;

                    if(is_lidar1_active == true && is_lidar2_active == true)
                    {
                        input_data_[lidar_index]->changePacketRate(FRAMERATE * PACKET_PER_SECOND * 2);
                        active_lidar_count = 2;
                    }
                    else
                    {
                        input_data_[lidar_index]->changePacketRate(FRAMERATE * PACKET_PER_SECOND * 1);
                        if(is_lidar1_active == true)
                            ++active_lidar_count;
                        if(is_lidar2_active == true)
                            ++active_lidar_count;
                    }

                    if(lidar_frame->packets.size() == active_lidar_count * PACKET_PER_SECOND)
                    {
                    
                        lidar_frame->lidar_index = lidar_index;
                        pub_frame_.publish(lidar_frame);
                    }
                    else
                    {
                    	ROS_INFO("skip publish : %d", lidar_frame->packets.size() ); 
                    }
                    lidar_frame.reset(new autol_msgs::AutolFrame);
                    lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);

                    lidar1_frame_count = 0;
                    lidar2_frame_count = 0;
                }
                
                is_packet_lost = false;
            }
        }

        delete packet;
        packet = NULL;
    }

}
