#include "node_lidar_ros.h"

#include "node_lidar.h"

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("cspc_lidar")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10, std::bind(&MinimalSubscriber::topic_callback, this,std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg) const
    {
		switch (msg->data)
		{
		
			case 1:
				node_lidar.lidar_status.lidar_ready = true;
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				printf("#start lidar\n");
				break;
		
			case 2:
				node_lidar.lidar_status.lidar_ready = false;
				node_lidar.lidar_status.close_lidar = true;
				node_lidar.serial_port->write_data(end_lidar,4);
				printf("#stop lidar\n");
				break;
			
			case 3:
				node_lidar.serial_port->write_data(high_exposure,4);
				break;
			
			case 4:
				node_lidar.serial_port->write_data(low_exposure,4);
				break;
			
			case 5:
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				break;
			
			case 6:
				node_lidar.serial_port->write_data(high_speed,4);
				node_lidar.lidar_general_info.frequency_max = 103;
				node_lidar.lidar_general_info.frequency_min = 97;
				break;
		
			case 7:
				node_lidar.serial_port->write_data(low_speed,4);
				node_lidar.lidar_general_info.frequency_max = 68;
				node_lidar.lidar_general_info.frequency_min = 52;
				break;
			
			default:
				break;
		}
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
};

/*处理最新一圈雷达的数据*/
int Scan_to_PointCloud(LaserScan &scan, sensor_msgs::msg::PointCloud2 &outscan)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->header.frame_id = "base_pcloud";
	cloud->width = scan.points.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	pcl::PointXYZ pcl_point;
	for(int i = 0;i<scan.points.size();i++)
	{
		pcl_point.x = scan.points[i].range * std::cos(scan.points[i].angle*M_PI/180.0);
		pcl_point.y = scan.points[i].range * std::sin(scan.points[i].angle*M_PI/180.0);
		pcl_point.z = 0;
		cloud->points[i] = pcl_point;
	}
	
	pcl::toROSMsg(*cloud, outscan);

}

int topic_thread()
{
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
}


int main(int argc, char **argv)
{
   
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("cspc_lidar");
	
	node->declare_parameter("port");
  	node->get_parameter("port", node_lidar.lidar_general_info.port);
	
	node->declare_parameter("baudrate");
  	node->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

	node->declare_parameter("frame_id");
  	node->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);
	
	node->declare_parameter("version");
  	node->get_parameter("version", node_lidar.lidar_general_info.version);
	//std::string frame_id = "base_scan";
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub;
	error_pub = node->create_publisher<std_msgs::msg::String>("lsd_error", 10);
	std_msgs::msg::String pubdata;

	thread t2(topic_thread);
	t2.detach();
	
	
	auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
	auto pcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

	node_start();
	
	while(rclcpp::ok())
	{
		if(node_lidar.lidar_status.lidar_abnormal_state != 0)
		{
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x01)
			{
				pubdata.data="node_lidar is trapped\n";
				error_pub->publish(pubdata);
				printf("异常状态1---trapped\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x02)
			{
				pubdata.data="node_lidar frequence abnormal\n";
				error_pub->publish(pubdata);
				printf("异常状态2---frequence abnormal\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x04)
			{
				pubdata.data="node_lidar is blocked\n";
				error_pub->publish(pubdata);
				printf("异常状态3---blocked\n");
			}
			node_lidar.serial_port->write_data(end_lidar,4);
			node_lidar.lidar_status.lidar_ready = false;

			sleep(1);
		}
		LaserScan scan;
		
		if(data_handling(scan))
		{
			auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();//雷达数据
			sensor_msgs::msg::PointCloud2 pclMsg; //雷达点云数据
			Scan_to_PointCloud(scan,pclMsg);

			scan_msg->ranges.resize(scan.points.size());
			scan_msg->intensities.resize(scan.points.size());

			scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);;
			scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
			scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;

			scan_msg->angle_min = scan.config.min_angle;
			scan_msg->angle_max = scan.config.max_angle;
			scan_msg->angle_increment = scan.config.angle_increment;
			scan_msg->scan_time = scan.config.scan_time;
			scan_msg->time_increment = scan.config.time_increment;
			scan_msg->range_min = scan.config.min_range;
			scan_msg->range_max = scan.config.max_range;
			for(int i=0; i < scan.points.size(); i++) {
				scan_msg->ranges[i] = scan.points[i].range;
				scan_msg->intensities[i] = scan.points[i].intensity;
			}
			//printf("publish--------\n");
			laser_pub->publish(*scan_msg);
			pcloud_pub->publish(pclMsg);
      	}
	}
	node_lidar.serial_port->write_data(end_lidar,4);
	return 0;
}