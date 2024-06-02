#ifndef POINT_CLOUD_OPTIMIZE_H
#define POINT_CLOUD_OPTIMIZE_H

//#include "node_lidar.h"
#include "lidar_data_processing.h"
#include <vector>
#include "lidar_information.h"

using namespace std;
class Point_cloud_optimize
{
private:
    int cnt=0;
    int cnt_all=0;
    int cnt_judge=0;
    int cnt_record=0;
    bool cumulation = false;
    short Lidar_Blocked[800];
    
public:

    /*点云滤波*/
    void PointCloudFilter(LaserScan *Scan);

    /*当前点的距离减去相同角度下雷达到扫地机边缘的距离*/
    int UltrasonicSimRanging(LaserPoint &pScan);

    /*获取需要剔除的角度信息*/
    void getLidarCoverAngle(char *charbuf);

    /*雷达被遮挡的判断*/
    void lidar_blocked_judge(int count);

    /*雷达被遮挡计数*/
    void lidar_blocked_count(LaserPoint &pScan,int count_lidar);

    /*点云缺口剔除*/
    void lidar_cover_cut(float &,float &);

    /*数据重置*/
    void datas_clear();

    void lidar_blocked_init();
    
};

#endif