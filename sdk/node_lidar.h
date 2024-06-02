#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H

#include <stdint.h>
#include <vector>
#include <string>
#include <atomic>
#include "point_cloud_optimize.h"
#include "lidar_data_processing.h"
#include "serial_port.h"
#include "locker.h"
#include "timer.h"
#include <memory>
#include <iostream>


//#include <ros/ros.h>
//#include <sensor_msgs/LaserScan.h>

#include <iostream>

using namespace std;

/*雷达基本信息*/
typedef struct{
  int version = 4; //雷达版本
  string port = "/dev/sc_mini";     //串口名称
  int m_SerialBaudrate = 230400;   //波特率
  bool m_intensities = false;
  uint64_t m_PointTime = 1e9/3800; 
  uint32_t trans_delay = 0;
  string frame_id = "laser_link";
  uint16_t frequency_max = 103;
  uint16_t frequency_min = 97;
}lidar_general_info_t;

/*雷达被遮挡判断结构体*/
typedef struct{
  uint16_t point_check = 0;
  uint16_t point_check_part = 0;
  uint16_t point_check_all = 0;
  int blocked_size = 0;
  int lidar_zero_count = 0;
  bool blocked_judge = true;
}lidar_block_t;

/*雷达原始数据包*/
typedef struct{
  node_package package;
  node_packages packages;
  node_package_coin package_coin;
}lidar_package_t;

/*雷达的时间*/
typedef struct{
  uint32_t scan_time_t = 0;      
  uint64_t scan_time_current = 0;
  uint64_t scan_time_record = 0;
  uint64_t system_start_time = 0;
  uint64_t last_encry_time = 0;
  uint64_t tim_scan_start = 0;
  uint64_t tim_scan_end = 0;
  uint64_t scan_start_time = 0;
  uint64_t scan_end_time = 0;
  uint64_t lidar_frequence_abnormal_time = 0;
}lidar_time_t;

/*机器人结构信息*/
typedef struct{
  float install_to_zero = 90.0;
  int ROBOT_DIAMETER_mm;               //扫地机直径
  int LIDAR_ROBOT_CENTER_DISTANCE_mm;  //扫地机雷达安装位置到中心的距离
  int LidarCoverBarNumber = 0;//雷达缺口剔除数值
  vector<LidarCoverAngleStr> LidarCoverAngle;
}lidar_robot_info_t;

/*雷达节点的运行状态*/
typedef struct{
  bool FilterEnable = true;           //是否需要滤波
  bool GyroCompensateEnable = false; //是否要做旋转角度补偿
  bool isConnected = false;          //串口连接状体
  bool slam_user = false;
  bool encry_lidar = false;
  bool optimize_enable = true;
  bool lidar_cover = false;
  bool disable_encry = false;
  bool able_exposure = false;
  bool low_exposure = false;
  bool lidar_ready = false;
  bool lidar_last_status =false;
  bool close_lidar = true;
  bool lidar_trap_restart = false;    //雷达卡住重启状态

  bool lidar_restart_try = false;


  uint8_t lidar_abnormal_state = 0; 
}lidar_status_t;

struct node_lidar_t
{

  node_info *scan_node_buf;      //存储激光雷达点的信息
  uint8_t *globalRecvBuffer;     //存储接收到的雷达数据
  lidar_block_t lidar_block;     //雷达被遮挡判断

  lidar_package_t scan_packages; //不同版本的雷达数据包

  lidar_time_t lidar_time;       //雷达的时间

  lidar_general_info_t lidar_general_info; //雷达基本信息

  lidar_robot_info_t lidar_robot_info;     //雷达相对机器人安装位置的信息

  lidar_status_t lidar_status;             //雷达运行状态

  size_t scan_node_count = 0;	//激光点数
  
  uint8_t recv_encryp[8];
  uint8_t lidar_version[8];

  Point_cloud_optimize optimize_lidar;
  Lidar_Data_Processing lidar_data_processing;
  //Serial_Port *serial_port;
  std::shared_ptr<Serial_Port> serial_port;
  Event _dataEvent;
  Locker _lock;

  bool data_calibration = false;
  float range_max;

  ~node_lidar_t();
};

extern node_lidar_t node_lidar;

bool data_handling(LaserScan &outscan);
bool initialize(); 
int node_start();
void send_lidar_data(LaserScan &outscan);
void flushSerial();
result_t grabScanData(uint32_t timeout = DEFAULT_TIMEOUT);
result_t connect(const char *port_path, uint32_t baudrate,node_lidar_t &lidar_arg);


#endif
