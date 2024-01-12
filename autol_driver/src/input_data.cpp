#include "autol_driver/input_data.h"

namespace autol_driver
{
    InputData::InputData(ros::NodeHandle private_nh, uint16_t port, double packet_rate):private_nh_(private_nh), port_(port), packet_rate_(ros::Rate(packet_rate)), is_ready_input_(false)
    {
    }
    
    SocketInput::SocketInput(ros::NodeHandle private_nh, uint16_t port):InputData(private_nh, port)
    {
        if(ConnectUdpCom() == true)
            is_ready_input_ = true;
    }
  
    bool SocketInput::ConnectUdpCom()
    {
        int ret = 0;
        if (ret == 0)
           ret = udp_socket.CreateSocket();
        if (ret == 0)
           ret = udp_socket.Bind(port_);

        if (ret < 0)
            ROS_INFO("Bind error %d", ret); 

        udp_socket.SetSocketBuffer(BUF_SIZE);       

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 3;

        udp_socket.SetTimeout(tv);

        if (ret < 0)
        {
            ROS_INFO("Failed UDP Connection"); 
            return false;
        }
        else
        {
            ROS_INFO("Success UDP Connection"); 
            return true;
        }
    }

    SocketInput::~SocketInput(void)
    {
        ROS_INFO("close SocketInput()"); 
        udp_socket.CloseSocket();
    }

    int SocketInput::getPacket(autol_msgs::AutolPacket* packet, int len)
    {
        return udp_socket.RecvFrom(packet, len);
    }

    PcapInput::PcapInput(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                       bool read_once, bool read_fast, double repeat_delay):InputData(private_nh, port, packet_rate), filename_(filename)
    {
        pcap_ = NULL;  
        empty_ = true;

        // get parameters using private node handle
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);

        if (read_once_)
            ROS_INFO("Read input file only once.");
        if (read_fast_)
            ROS_INFO("Read input file as quickly as possible.");
        if (repeat_delay_ > 0.0)
            ROS_INFO("Delay %.3f seconds before repeating input file.",repeat_delay_);

        // Open the PCAP dump file
        ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
        {
            ROS_FATAL("Error opening AutoL socket dump file.");
            return;
        }
        else
        {
            is_ready_input_ = true;
        }

        std::stringstream filter;
        if( devip_str_ != "" )              // using specific IP?
        {
            filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

    /** destructor */
    PcapInput::~PcapInput(void)
    {
        pcap_close(pcap_);
    }
  
    void PcapInput::changePacketRate(double packet_rate)
    {
        packet_rate_ = ros::Rate(packet_rate);
    }

    int PcapInput::getPacket(autol_msgs::AutolPacket *pkt, int len)
    {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;

        int packet_cnt = 0;
        while (true)
        {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
            {
                // Skip packets not for the correct port and from the
                // selected IP address.
                int ret = pcap_offline_filter(&pcap_packet_filter_, header, pkt_data);

                if (0 == ret)
                    continue;
                
                // Keep the reader from blowing through the file.
                if (read_fast_ == false)
                    packet_rate_.sleep();
                
                memcpy(&pkt->data[0], pkt_data+42, PACKET_DATA_SIZE);

                empty_ = false;
                return len;                   // success
            }

            if (empty_)                 // no data in file?
            {
                ROS_WARN("Error %d reading Velodyne packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }

            if (read_once_)
            {
                ROS_INFO("end of file reached -- done reading.");
                return -1;
            }

            if (repeat_delay_ > 0.0)
            {
                ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }

            ROS_DEBUG("replaying AutoL dump file");
            
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
        }

        return 0;
    }
}
