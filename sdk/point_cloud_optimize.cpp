#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "node_lidar.h"
#include "point_cloud_optimize.h"


/*****************************************
*             点云滤波函数                 *
******************************************/
void Point_cloud_optimize::PointCloudFilter(LaserScan *Scan)
{
	int i0, i1, i2, i3, i4;
	float x0, y0, x1, y1, x2, y2, x3, y3, r1, r2;
	float d01, d12, d23, d34;
	float a, b, Adeta;
	float FilterRatioAdj;
	float AngDiff;
	int DepthState;
	float FilterRatio = 2.5;

	AngDiff = 2 * PI / (Scan->points.size() - 1);
	FilterRatioAdj = FilterRatio * sin(AngDiff);

	for (i0 = 0; i0 < Scan->points.size(); i0++)
	{
		i1 = i0 + 1;
		i1 = i1 < Scan->points.size() ? i1 : i1 - Scan->points.size();
		i2 = i0 + 2;
		i2 = i2 < Scan->points.size() ? i2 : i2 - Scan->points.size();
		i3 = i0 + 3;
		i3 = i3 < Scan->points.size() ? i3 : i3 - Scan->points.size();
		DepthState = 0;
		DepthState += (Scan->points[i0].range != 0) * 1;
		DepthState += (Scan->points[i1].range != 0) * 2;
		DepthState += (Scan->points[i2].range != 0) * 4;
		DepthState += (Scan->points[i3].range != 0) * 8;
		if (DepthState == 0x0F)
		{ //1111
			x0 = Scan->points[i0].range * cos(Scan->points[i0].angle * PI / 180);
			y0 = Scan->points[i0].range * sin(Scan->points[i0].angle * PI / 180);
			x3 = Scan->points[i3].range * cos(Scan->points[i3].angle * PI / 180);
			y3 = Scan->points[i3].range * sin(Scan->points[i3].angle * PI / 180);
			x1 = (x3 + 2 * x0) / 3;
			y1 = (y3 + 2 * y0) / 3;
			x2 = (2 * x3 + x0) / 3;
			y2 = (2 * y3 + y0) / 3;
			r1 = sqrt(x1 * x1 + y1 * y1);
			r2 = sqrt(x2 * x2 + y2 * y2);

			a = Scan->points[i0].range;
			b = Scan->points[i1].range;
			Adeta = fabs(Scan->points[i0].angle - Scan->points[i1].angle) * PI / 180;
			d01 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
			a = Scan->points[i1].range;
			b = Scan->points[i2].range;
			Adeta = fabs(Scan->points[i1].angle - Scan->points[i2].angle) * PI / 180;
			d12 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
			a = Scan->points[i2].range;
			b = Scan->points[i3].range;
			Adeta = fabs(Scan->points[i2].angle - Scan->points[i3].angle) * PI / 180;
			d23 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

			if (FilterRatioAdj * Scan->points[i1].range < d01 && FilterRatioAdj * Scan->points[i1].range < d12)
			{
				Scan->points[i1].range = 0;
			}
			if (FilterRatioAdj * Scan->points[i2].range < d12 && FilterRatioAdj * Scan->points[i2].range < d23)
			{
				Scan->points[i2].range = 0;
			}
			if (0 == Scan->points[i1].range || 0 == Scan->points[i2].range)
			{
				continue;
			}

			if ((Scan->points[i1].range > r1 && Scan->points[i2].range < r2) || (Scan->points[i1].range < r1 && Scan->points[i2].range > r2))
			{
				if (
					(d01 > FilterRatioAdj * Scan->points[i0].range && d12 < FilterRatioAdj * Scan->points[i1].range && d23 < FilterRatioAdj * Scan->points[i2].range) || (d01 < FilterRatioAdj * Scan->points[i0].range && d12 > FilterRatioAdj * Scan->points[i1].range && d23 < FilterRatioAdj * Scan->points[i2].range) || (d01 < FilterRatioAdj * Scan->points[i0].range && d12 < FilterRatioAdj * Scan->points[i1].range && d23 > FilterRatioAdj * Scan->points[i2].range))
				{
				}
				else
				{
					Scan->points[i1].range = r1 + 0.4 * (Scan->points[i1].range - r1);
					Scan->points[i2].range = r2 + 0.4 * (Scan->points[i2].range - r2);
				}
			}
			else
			{
			}
		}
		else if (DepthState == 0x00 || DepthState == 0x08 || DepthState == 0x01)
		{ // 0000 1000 0001
		}
		else if ((DepthState & 0x0E) == 0x04)
		{ //010x
			Scan->points[i2].range = 0;
		}
		else if ((DepthState & 0x07) == 0x02)
		{ //x010
			Scan->points[i1].range = 0;
		}
		else if ((DepthState & 0x07) == 0x03)
		{ //x011
			a = Scan->points[i0].range;
			b = Scan->points[i1].range;
			if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) > FilterRatioAdj * Scan->points[i1].range)
			{
				Scan->points[i1].range = 0;
			}
		}
		else if (DepthState == 0x0E)
		{ //1110
			a = Scan->points[i1].range;
			b = Scan->points[i2].range;
			if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) > FilterRatioAdj * Scan->points[i1].range)
			{
				Scan->points[i1].range = 0;
			}
		}
		else if ((DepthState & 0x0E) == 0x0C)
		{ //110x 0111
			a = Scan->points[i2].range;
			b = Scan->points[i3].range;
			if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) > FilterRatioAdj * Scan->points[i2].range)
			{
				Scan->points[i2].range = 0;
			}
		}
		else if (DepthState == 0x07)
		{ //0111
			a = Scan->points[i1].range;
			b = Scan->points[i2].range;
			if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) > FilterRatioAdj * Scan->points[i2].range)
			{
				Scan->points[i2].range = 0;
			}
		}
		else if (DepthState == 0x06)
		{ //0110
			a = Scan->points[i1].range;
			b = Scan->points[i2].range;
			if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) > FilterRatioAdj * Scan->points[i1].range)
			{
				Scan->points[i1].range = 0;
				Scan->points[i2].range = 0;
			}
		}
		else
		{
		}
	}

	for (i0 = 0; i0 < Scan->points.size(); i0++)
	{
		i1 = i0 + 1;
		i1 = i1 < Scan->points.size() ? i1 : i1 - Scan->points.size();
		i2 = i0 + 2;
		i2 = i2 < Scan->points.size() ? i2 : i2 - Scan->points.size();
		i3 = i0 + 3;
		i3 = i3 < Scan->points.size() ? i3 : i3 - Scan->points.size();
		i4 = i0 + 4;
		i4 = i4 < Scan->points.size() ? i4 : i4 - Scan->points.size();
		if (Scan->points[i0].range != 0 && Scan->points[i1].range != 0 && Scan->points[i2].range != 0 && Scan->points[i3].range != 0 && Scan->points[i4].range != 0)
		{
			a = Scan->points[i0].range;
			b = Scan->points[i1].range;
			Adeta = fabs(Scan->points[i0].angle - Scan->points[i1].angle) * PI / 180;
			d01 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

			a = Scan->points[i1].range;
			b = Scan->points[i2].range;
			Adeta = fabs(Scan->points[i1].angle - Scan->points[i2].angle) * PI / 180;
			d12 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

			a = Scan->points[i2].range;
			b = Scan->points[i3].range;
			Adeta = fabs(Scan->points[i2].angle - Scan->points[i3].angle) * PI / 180;
			d23 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

			a = Scan->points[i3].range;
			b = Scan->points[i4].range;
			Adeta = fabs(Scan->points[i3].angle - Scan->points[i4].angle) * PI / 180;
			d34 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

			if (
				d01 < FilterRatioAdj * Scan->points[i1].range && d34 < FilterRatioAdj * Scan->points[i3].range && (d12 > FilterRatioAdj * Scan->points[i2].range || d23 > FilterRatioAdj * Scan->points[i2].range))
			{
				if (
					(Scan->points[i0].range < Scan->points[i1].range && Scan->points[i3].range < Scan->points[i4].range) || (Scan->points[i0].range > Scan->points[i1].range && Scan->points[i3].range > Scan->points[i4].range))
				{
					Scan->points[i2].range = (Scan->points[i1].range + Scan->points[i3].range) / 2;
				}
			}
		}
		else if (Scan->points[i0].range == 0 && Scan->points[i1].range != 0 && Scan->points[i2].range != 0 && Scan->points[i3].range != 0 && Scan->points[i4].range != 0)
		{
			x2 = Scan->points[i2].range * cos(Scan->points[i2].angle * PI / 180);
			y2 = Scan->points[i2].range * sin(Scan->points[i2].angle * PI / 180);
			x3 = Scan->points[i3].range * cos(Scan->points[i3].angle * PI / 180);
			y3 = Scan->points[i3].range * sin(Scan->points[i3].angle * PI / 180);
			x1 = 2 * x2 - x3;
			y1 = 2 * y2 - y3;
			Scan->points[i1].range = Scan->points[i1].range + 0.4 * (sqrt(x1 * x1 + y1 * y1) - Scan->points[i1].range);
		}
		else if (Scan->points[i0].range != 0 && Scan->points[i1].range != 0 && Scan->points[i2].range != 0 && Scan->points[i3].range != 0 && Scan->points[i4].range == 0)
		{
			x1 = Scan->points[i1].range * cos(Scan->points[i1].angle * PI / 180);
			y1 = Scan->points[i1].range * sin(Scan->points[i1].angle * PI / 180);
			x2 = Scan->points[i2].range * cos(Scan->points[i2].angle * PI / 180);
			y2 = Scan->points[i2].range * sin(Scan->points[i2].angle * PI / 180);
			x3 = 2 * x2 - x1;
			y3 = 2 * y2 - y1;
			Scan->points[i3].range = Scan->points[i3].range + 0.4 * (sqrt(x3 * x3 + y3 * y3) - Scan->points[i3].range);
		}
		else
		{
		}
	}
}

/*****************************************
* 雷达实际测距距离-雷达安装位置到扫地机边缘的距离*
******************************************/
int Point_cloud_optimize::UltrasonicSimRanging(LaserPoint &pScan)
{
	int i, index;
	short tempdep;
	float target_angle = pScan.angle + node_lidar.lidar_robot_info.install_to_zero;

	if(target_angle > 360.0)
	{
		target_angle = (target_angle-360.0);
	}
	if(target_angle > 360.0 || target_angle < 0.0)
	{
		target_angle = 0;
	}
	index = (int)(target_angle/ULTRASONIC_ANGLE_INC_DEG + 0.5);

	if (index < node_lidar.lidar_block.blocked_size)
	{
		if(pScan.range==0)
		{
			pScan.range_check = 0;
		}else{
			pScan.range_check = pScan.range - Lidar_Blocked[index];
		}
	}else{
		pScan.range_check = 0;
	}
}

/*****************************************
*         获取结构缺口剔除的角度             *
******************************************/
void Point_cloud_optimize::getLidarCoverAngle(char *charbuf)
{
	LidarCoverAngleStr tempAngle;
	char tempstr[ANGLESTRLENMAX];
	int searstate = 0, index; 

	int len = strlen(charbuf);
	node_lidar.lidar_robot_info.LidarCoverBarNumber = 0;
	for (int i = 0; i < len; i++)
	{
		if (charbuf[i] == '[')
		{
			searstate = 1;
			index = 0;
			memset(tempstr, 0, ANGLESTRLENMAX);
		}
		else if (charbuf[i] == ',' && strlen(tempstr) > 0)
		{
			searstate = 2;
			index = 0;
			tempAngle.f_begin = atof(tempstr);
			memset(tempstr, 0, ANGLESTRLENMAX);
		}
		else if (charbuf[i] == ']' && searstate == 2 && strlen(tempstr) > 0)
		{
			tempAngle.f_end = atof(tempstr);
			if (tempAngle.f_begin < tempAngle.f_end && tempAngle.f_end <= 360.0)
			{
				node_lidar.lidar_robot_info.LidarCoverBarNumber++;
				node_lidar.lidar_robot_info.LidarCoverAngle.push_back(tempAngle);
			}
			searstate = 0;
		}
		else if (searstate != 0)
		{
			tempstr[index++] = charbuf[i];
			index = (index >= ANGLESTRLENMAX) ? (ANGLESTRLENMAX - 1) : index;
		}
		else
		{
		}
	}
}


void Point_cloud_optimize::lidar_blocked_count(LaserPoint &pScan,int count_lidar)
{
	int cnt_judge_count = count_lidar / (2*7);
	if(pScan.range_check < -10 && pScan.range_check > -200)
	{
		cnt_all++;
		if(abs(cnt_judge-cnt_record) < 20)
		{
			cnt++;
		}else{
			cnt = 0;
		}
		if(cnt > cnt_judge_count)
		{
			cumulation = true;
		}
		cnt_record = cnt_judge;
	}
	cnt_judge++;
}


/*****************************************
*         雷达被遮挡判断             		*
******************************************/
void Point_cloud_optimize::lidar_blocked_judge(int count)
{
	if((float)node_lidar.lidar_block.lidar_zero_count / (float)count > 0.70)
	{
		node_lidar.lidar_block.point_check++;
	}else{
		if(node_lidar.lidar_block.point_check > 0)
		{
			node_lidar.lidar_block.point_check--;
		}
	}
	if(node_lidar.lidar_block.point_check > 200)
	{
		printf("lidar 雷达被严重遮挡百分之七十 %d,%d\n",node_lidar.lidar_block.lidar_zero_count,node_lidar.scan_node_count);
		node_lidar.lidar_status.lidar_abnormal_state |= 0x04;
		node_lidar.lidar_block.point_check = 0;
	}
	datas_clear();
}


/*****************************************
*         雷达点云缺口剔除             		*
******************************************/
void Point_cloud_optimize::lidar_cover_cut(float &range,float &angle)
{
	for (int k = 0; k < node_lidar.lidar_robot_info.LidarCoverBarNumber; k++)
	{
		if (node_lidar.lidar_robot_info.LidarCoverAngle[k].f_begin < angle && angle < node_lidar.lidar_robot_info.LidarCoverAngle[k].f_end)
		{
			range = 0;
		}
	}
}

void Point_cloud_optimize::datas_clear()
{
	cnt=0;
    cnt_all=0;
    cnt_judge=0;
    cnt_record=0;
    cumulation = false;
}

void Point_cloud_optimize::lidar_blocked_init()
{
	float angle,radian;
	float r, d, temp;

	r = node_lidar.lidar_robot_info.ROBOT_DIAMETER_mm / 2.0F;
	d = node_lidar.lidar_robot_info.LIDAR_ROBOT_CENTER_DISTANCE_mm * 1.0;

	float x,y;
	float k,s,a,b,c;

	int size = 360/ULTRASONIC_ANGLE_INC_DEG;
	node_lidar.lidar_block.blocked_size = size;

	for (int j = 0; j < size; j++)
	{
		angle = (360.0 - j * ULTRASONIC_ANGLE_INC_DEG);
		radian = angle * PI / 180.0;
		if(angle == 90.0)
		{
			Lidar_Blocked[j] = r-d;
		}else if(angle == 270)
		{
			Lidar_Blocked[j] = r+d;
		}
		else{
			radian = fabs(radian);
			k = tan(radian);
			a=1+k*k;
			b=2*d*k;
			c=d*d-r*r; 
			s=b*b-4*a*c;
			if((angle >= 0 && angle < 90)|| angle > 270 && angle <=360){
				x=(-1*b+sqrt(s))/(2*a);
			}else{
				x=(-1*b-sqrt(s))/(2*a);
			}
			y=k*x+d;
			Lidar_Blocked[j] = sqrt(x*x+(y-d)*(y-d));
		}
	}
}


