#include <iostream>
#include <cstring>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <cmath>

#include "calibration.h"

using namespace std;

calibration_params_t calibration_params;

#define RADIAN360 6.28318530717948
#define RADIAN180 3.14159265358974

/*弧度转角度*/
float RADIAN2ANGLE(float x)
{
  return 57.29577951308232 * x;
}
/*角度转弧度*/
float ANGLE2RADIAN(float x)
{
  return 0.017453292519943 * x;
}

/*点到点的距离*/
float pointTopointDist(point_t a, point_t b)
{
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/*清空点云数据*/
void pcloud_clear(pcloud_t &pcloud)
{
  pcloud.data.clear();
}

/*将雷达数据转换为点云数据*/
void pcloud_build_by_scan(LaserScan &outscan, pcloud_t &pcloud)
{
  point_t point;
  for (int i = 0; i < outscan.points.size(); i++)
  {
    memset(&point, 0, sizeof(point_t));
    // 阈值滤波
    // if (scan->lasers[i].dis < lidar_min_range || scan->lasers[i].dis > lidar_max_range || scan->lasers[i].quality)
    if (outscan.points[i].range * 1000 <= 0 || outscan.points[i].range * 1000 > 10 * 1000)
    {
      outscan.points[i].range = 0;
    }
    // 距离/角度转换点
    point.index = i;
    point.x = cosf(ANGLE2RADIAN(-outscan.points[i].angle)) * outscan.points[i].range * 1000;
    point.y = sinf(ANGLE2RADIAN(-outscan.points[i].angle)) * outscan.points[i].range * 1000;

    point.angle = outscan.points[i].angle;
    point.intensity = outscan.points[i].intensity;
    point.range = outscan.points[i].range;

    pcloud.data.emplace_back(point);
  }
}

/*将极坐标系下的点云转换到世界坐标系下*/
void pcloud_offset_by_pos(pcloud_t &pcloud)
{
  pos_t lidar_pos = pcloud.origin;

  point_t *point;
  float x, y;
  int size = (int)pcloud.data.size();
  for (int i = 0; i < size; i++)
  {
    point = &pcloud.data[i];
    x = point->x;
    y = point->y;

    point->x = lidar_pos.x + cosf(lidar_pos.theta) * x - sinf(lidar_pos.theta) * y;
    point->y = lidar_pos.y + cosf(lidar_pos.theta) * y + sinf(lidar_pos.theta) * x;
  }
}

/*点云转换函数*/
void pcloud_build_by_pos_scan(LaserScan &outscan, pcloud_t &pcloud)
{
  pcloud_clear(pcloud);

  pcloud.origin.x = calibration_params.origin_point.x;
  pcloud.origin.y = calibration_params.origin_point.y;
  pcloud.origin.theta = 0;

  // 将雷达原始数据转换成极坐标点云
  pcloud_build_by_scan(outscan, pcloud);

  // 将极坐标点云转换成真实坐标系下的点云
  pcloud_offset_by_pos(pcloud);
}

/*查找对应点*/
int find_index(point_t point, pcloud_t &pcloud)
{
  point_t point_min;
  memset(&point_min, 0, sizeof(point_t));
  float legth = 15000;
  for (auto p_point : pcloud.data)
  {
    double distance = sqrt(pow(p_point.x - point.x, 2) + pow(p_point.y - point.y, 2));

    if (distance < legth)
    {
      legth = distance;
      point_min = p_point;
    }
    /*
    if(p_point.x == point.x && p_point.y == point.y)
    {
      return p_point.index;
    }*/
  }
  // std::cout <<"p-x:"<< point.x << "p-y:" << point.y << std::endl;
  // std::cout <<"min-x:"<< point_min.x << "min-y:" << point_min.y << std::endl;
  if (point_min.index > pcloud.data.size())
  {
    return 0;
  }
  else
  {
    return point_min.index;
  }
}

// 判断点是否在线段左侧
int isLeft(point_t p, point_t a, point_t b)
{
  double cross_product = a.x * b.y + b.x * p.y + p.x * a.y - a.x * p.y - b.x * a.y - p.x * b.y;
  if (cross_product > 0)
    return 1;
  else if (cross_product < 0)
    return 2;
  else
    return 0;
}

// 判断两个线段是否相交，并计算交点
bool intersect(LineSegment line1, LineSegment line2, Vector2D &intersection)
{
  // 计算两个方向向量
  Vector2D direction1 = {line1.p2.x - line1.p1.x, line1.p2.y - line1.p1.y};
  Vector2D direction2 = {line2.p2.x - line2.p1.x, line2.p2.y - line2.p1.y};

  // 计算向量叉积
  double cross = direction1.x * direction2.y - direction1.y * direction2.x;

  // 如果叉积为0，说明两个线段共线
  if (cross == 0)
  {
    return false;
  }

  // 计算线段起点位置向量
  Vector2D position1 = {line2.p1.x - line1.p1.x, line2.p1.y - line1.p1.y};
  Vector2D position2 = {line1.p1.x - line2.p1.x, line1.p1.y - line2.p1.y};

  // 计算向量叉积
  double cross1 = position1.x * direction1.y - position1.y * direction1.x;
  double cross2 = position2.x * direction2.y - position2.y * direction2.x;

  // 计算交点坐标
  intersection.x = line1.p1.x + direction1.x * cross2 / cross;
  intersection.y = line1.p1.y + direction1.y * cross2 / cross;

  // 判断交点是否在两个线段上
  /*
  if (intersection.x < min(line1.p1.x, line1.p2.x) || intersection.x > max(line1.p1.x, line1.p2.x) ||
      intersection.x < min(line2.p1.x, line2.p2.x) || intersection.x > max(line2.p1.x, line2.p2.x)) {
      return false;
  }

  if (intersection.y < min(line1.p1.y, line1.p2.y) || intersection.y > max(line1.p1.y, line1.p2.y) ||
      intersection.y < min(line2.p1.y, line2.p2.y) || intersection.y > max(line2.p1.y, line2.p2.y)) {
      return false;
  }*/

  // 两个线段相交，返回true
  return true;
}

void straightening_process(point_lines line_point, point_t a, point_t b, pcloud_t &pcloud)
{
  LineSegment line1 = {{line_point.line_start.x, line_point.line_start.y}, {line_point.line_end.x, line_point.line_end.y}};
  LineSegment line2 = {{(double)b.x, (double)b.y}, {(double)a.x, (double)a.y}};
  Vector2D intersection;
  if (intersect(line1, line2, intersection))
  {
    cout << "线段交点坐标为：(" << intersection.x << ", " << intersection.y << ")" << endl;
  }
  else
  {
    cout << "线段不相交" << endl;
  }
  float dist = sqrt((a.x - intersection.x) * (a.x - intersection.x) + (a.y - intersection.y) * (a.y - intersection.y));

  if (dist < 100)
  {
    pcloud.data[a.index].x = intersection.x;
    pcloud.data[a.index].y = intersection.y;
    pcloud.data[a.index].range = sqrt((b.x - intersection.x) * (b.x - intersection.x) + (b.y - intersection.y) * (b.y - intersection.y));
  }
}

void straightening_process_new(std::vector<point_t> line_point, point_t p_point, point_t origin_point, pcloud_t &pcloud)
{
  
  point_t line_start_point = line_point.front();
  point_t line_end_point = line_point.back();

  /*
  LineSegment line1 = {{line_start_point.x, line_start_point.y}, {line_end_point.x, line_end_point.y}};
  LineSegment line2 = {{(double)b.x, (double)b.y}, {(double)a.x, (double)a.y}};
  Vector2D intersection;
  if (intersect(line1, line2, intersection))
  {
    cout << "线段交点坐标为：(" << intersection.x << ", " << intersection.y << ")" << endl;
  }
  else
  {
    cout << "线段不相交" << endl;
  }*/
  Line_new line1, line2;

  double k, b;
  getLineKB(line_start_point, line_end_point, k, b);
  line1.slope = k;
  line1.intercept = b;

  getLineKB(p_point, origin_point, k, b);

  line2.slope = k;
  line2.intercept = b;

  point_t intersection = calculateIntersection(line1, line2);

  float dist = sqrt((p_point.x - intersection.x) * (p_point.x - intersection.x) + (p_point.y - intersection.y) * (p_point.y - intersection.y));
  cout << "拉直处理" << dist << endl;
  if (fabs(dist) < 200)
  {

    pcloud.data[p_point.index].x = intersection.x;
    pcloud.data[p_point.index].y = intersection.y;
    pcloud.data[p_point.index].range = sqrt((origin_point.x - intersection.x) * (origin_point.x - intersection.x) + (origin_point.y - intersection.y) * (origin_point.y - intersection.y));
  }
}

point_t calculateIntersection(Line_new line1, Line_new line2)
{
  point_t intersection;
  intersection.x = (line2.intercept - line1.intercept) / (line1.slope - line2.slope);
  intersection.y = line1.slope * intersection.x + line1.intercept;
  return intersection;
}

void Point_cloud_straightening_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud)
{
  point_t line_first_point1 = line_first.front();
  point_t line_first_point2 = line_first.back();

  point_t line_second_point1 = line_second.front();
  point_t line_second_point2 = line_second.back();

  // LineSegment line1 = { {line_first_point1.x, line_first_point1.y}, {line_first_point2.x, line_first_point2.y} };
  // LineSegment line2 = { {line_second_point1.x, line_second_point1.y}, {line_second_point2.x, line_second_point2.y} };

  // Vector2D intersection;
  Line_new line1, line2;

  double k, b;
  getLineKB(line_first_point1, line_first_point2, k, b);
  line1.slope = k;
  line1.intercept = b;

  getLineKB(line_second_point1, line_second_point2, k, b);

  line2.slope = k;
  line2.intercept = b;

  point_t intersection = calculateIntersection(line1, line2);

  getLineKB(calibration_params.origin_point, intersection, k, b);

  // pointToLineDist

  /*
  if (intersect(line1, line2, intersection)) {
      //cout << "线段交点坐标为：(" << intersection.x << ", " << intersection.y << ")" << endl;
  }
  else {
      cout << "线段不相交" << endl;
      return;
  }*/

  point_t cross_point;
  cross_point.x = intersection.x;
  cross_point.y = intersection.y;
  
  if (pointTopointDist(cross_point, calibration_params.origin_point) > calibration_params.distortion_processing_distance)
  {
    std::cout << "距离判断" << pointTopointDist(cross_point, calibration_params.origin_point)
         << "," << calibration_params.distortion_processing_distance << std::endl;
    return;
  }

  double legth = 50000;
  point_t point_min;
  point_min.index = -1;
  // int index = find_index(cross_point,pcloud);
  for (auto p_point : pcloud.data)
  {
    // double distance = sqrt(pow(p_point.x - point.x, 2) + pow(p_point.y - point.y, 2));
    if(p_point.x == 10000 && p_point.y ==10000)
    {
      continue;
    }
    float distance = pointToLineDist(p_point, k, b);
    if (fabs(distance) < fabs(legth))
    {
      legth = distance;
      point_min = p_point;
    }
    /*
    if(p_point.x == point.x && p_point.y == point.y)
    {
      return p_point.index;
    }*/
  }
  if(point_min.index == -1)
  {
    return;
  }
  int index = point_min.index;

  std::cout << "距离最近点的坐标 " << pcloud.data[index].x << "," << pcloud.data[index].y << std::endl;

  int start_index = (index - 3) > 0 ? (index - 3) : 0;

  if (calibration_params.calibration_type == 0)
  {
    for (int i = start_index; i < index + 3; i++)
    {
      pcloud.data[i].range = 0;
      pcloud.data[i].intensity = 0;
      pcloud.data[i].x = pcloud.origin.x;
      pcloud.data[i].y = pcloud.origin.y;
      pcloud.data[i].change = true;
    }
  }
  else if (calibration_params.calibration_type == 1)
  {
    for (int i = start_index; i < index + 3; i++)
    {
      int judge = isLeft(pcloud.data[i], calibration_params.origin_point, cross_point);
      if (judge == 1)
      {
        straightening_process_new(line_first, pcloud.data[i], calibration_params.origin_point, pcloud);
      }
      else if (judge == 2)
      {
        straightening_process_new(line_second, pcloud.data[i], calibration_params.origin_point, pcloud);
      }
      pcloud.data[i].change = true;
    }
  }
}

void neighbor_lines_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud)
{
  point_t line_first_point1 = line_first.front();
  point_t line_first_point2 = line_first.back();

  point_t line_second_point1 = line_second.front();
  point_t line_second_point2 = line_second.back();

  LineSegment line1 = {{line_first_point1.x, line_first_point1.y}, {line_first_point2.x, line_first_point2.y}};
  LineSegment line2 = {{line_second_point1.x, line_second_point1.y}, {line_second_point2.x, line_second_point2.y}};

  // 计算两个线段的方向向量
  Vector2D direction1 = {line1.p2.x - line1.p1.x, line1.p2.y - line1.p1.y};
  Vector2D direction2 = {line2.p2.x - line2.p1.x, line2.p2.y - line2.p1.y};

  // 求解两个方向向量的夹角
  double angle = direction1.Angle(direction2);

  // 将弧度转换为角度
  double degree = angle * 180 / M_PI;
  cout << "两条线段的夹角为：" << degree << "度" << endl;

  if (fabs(degree) > 110 && fabs(degree) < 70)
  {
    // remove_noise_point(line_f,line_s,pcloud);
  }
  else
  {
    Point_cloud_straightening_new(line_first, line_second, pcloud);
  }
}

void getLineKB(point_t start_point, point_t end_point, double &k, double &b)
{
  if (start_point.x == end_point.x)
  {
    // 直线垂直于x轴，斜率不存在
    k = NAN;
  }
  else if (start_point.y == end_point.y)
  {
    // 直线平行于x轴，斜率为0
    k = 0.0;
  }
  else
  {
    // 计算斜率和截距
    k = (end_point.y - start_point.y) / (end_point.x - start_point.x);
  }
  b = start_point.y - k * start_point.x;
}

// 求某点到直线的垂线距离
double pointToLineDist(point_t point, double k, double b)
{
  // 判断直线是否垂直于x轴
  if (fabs(k) < 1e-6)
  {
    return fabs(point.y - b);
  }

  // 计算直线上一个任意点的坐标
  double x0 = 0.0;
  double y0 = k * x0 + b;

  // 计算垂线斜率和截距
  double kt = -1.0 / k;
  double bt = point.y - kt * point.x;

  // 计算垂线与直线的交点坐标
  double ix = (bt - b) / (k - kt);
  double iy = k * ix + b;

  // 计算点到直线的距离
  return sqrt(pow(point.x - ix, 2.0) + pow(point.y - iy, 2.0));
}

bool line_judgment(std::vector<point_t> line, point_t point)
{
  point_t start_point = line.front();
  point_t end_point = line.back();
  double k, b;
  int reference = 10;
  static point_t reference_point;
  double d = 0;

  if (line.size() <= reference)
  {
    reference_point = end_point;
  }

  if (line.size() <= reference)
  {
    getLineKB(start_point, end_point, k, b);
    
    for (int i = 1; i < line.size() - 1; i++)
    {
      if(isnan(k))
      {
        d = line[i].x - start_point.x;
      }else if (k == 0.0){
        d = line[i].y - start_point.y;
      }else{
        d = pointToLineDist(line[i], k, b);
      }
      if (fabs(d) > 50)
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    getLineKB(start_point, reference_point, k, b);

    /*求线段夹角*/
    LineSegment line1 = {{start_point.x, start_point.y}, {reference_point.x, reference_point.y}};
    LineSegment line2 = {{start_point.x, start_point.y}, {point.x, point.y}};
    Vector2D direction1 = {line1.p2.x - line1.p1.x, line1.p2.y - line1.p1.y};
    Vector2D direction2 = {line2.p2.x - line2.p1.x, line2.p2.y - line2.p1.y};
    double angle = direction1.Angle(direction2);
    double degree = angle * 180 / M_PI;

    if(isnan(k))
    {
      d = point.x - start_point.x;
    }else if (k == 0.0){
      d = point.y - start_point.y;
    }else{
      d = pointToLineDist(point, k, b);
    }

    // std::cout << "a坐标x：" << start_point.x << "坐标y：" << start_point.y <<std::endl;
    // std::cout << "b坐标x：" << reference_point.x << "坐标y：" <<reference_point.y <<std::endl;
    // std::cout << "c坐标x：" << point.x << "坐标y：" << point.y <<std::endl;
    // std::cout << "距离：" << dd << "角度：" << degree <<"line size"<<line.size() <<std::endl;

    if (fabs(d) < 30 && fabs(degree) < 30)
    {
      return true;
    }else{
      return false;
    }
  }
}

void fitting_line(pcloud_t &pcloud)
{
  std::vector<point_t> line;               // 当前线段
  std::vector<point_t> line_append;        // 可能遗漏的线段
  std::vector<std::vector<point_t>> lines; // 全部线段

  point_t point_record1;
  point_t point_record2;
  point_t point_last_one = pcloud.data.back();

  // 遍历点云
  for (auto &point : pcloud.data)
  {
    // 超过3个点进入直线拟合流程
    if (line.size() >= 3)
    {
      // 直线拟合判断
      if (line_judgment(line, point) == false)
      {
        if (line.size() >= calibration_params.line_judgment_point_num)
        {
          if (lines.empty())
          {
            point_record1 = line.front();
          }
          else
          {
            point_record2 = line.back();
          }
          lines.emplace_back(line);
          line.clear();
        }
        else
        {
          line.clear();
        }
      }else{
        if(point.index == point_last_one.index && line.size() >= calibration_params.line_judgment_point_num)
        {
          lines.emplace_back(line);
        }
      }
    }
    line.emplace_back(point);
  }
  line.clear();

  point_t point_append,point_append_last;
  if(point_record2.index < pcloud.data.size() 
  &&  point_last_one.index < pcloud.data.size()
  &&  point_record1.index < pcloud.data.size())
  {
     if (point_record2.index + 1 < point_last_one.index)
    {
      for (int i = point_record2.index + 1; i < point_last_one.index; i++)
      {
        point_append = pcloud.data[i];
        line_append.emplace_back(point_append);
      }
    }
    if (point_record1.index != 0)
    {
      for (int i = 0; i < point_record1.index; i++)
      {
        point_append = pcloud.data[i];
        line_append.emplace_back(point_append);
      }
    }                                                                    
  }
 
  if(!line_append.empty())
  {
    point_append_last = line_append.back();

    if (line_append.size() >= calibration_params.line_judgment_point_num)
    {
      for (auto &point : line_append)
      {
        if (line.size() >= 3)
        {
          // 直线拟合判断
          if (line_judgment(line, point) == false)
          {
            if (line.size() >= calibration_params.line_judgment_point_num)
            {
              lines.emplace_back(line);
              line.clear();
            }
            else
            {
              line.clear();
            }
          }
        }else{
          if(point.index == point_append_last.index && line.size() >= calibration_params.line_judgment_point_num)
          {
            lines.emplace_back(line);
          }
        }
        line.emplace_back(point);
      }
    }
  }

  std::cout << "------###----line_size:" << lines.size() << std::endl;

  if (lines.size() >= 2)
  {
    neighbor_lines_new(lines.back(), lines.front(), pcloud);
    for (int i = 0; i < lines.size() - 1; i++)
    {
      neighbor_lines_new(lines[i], lines[i + 1], pcloud);
    }
  }
}

void lidar_calibration(LaserScan &outscan)
{
  pcloud_t pcloud;

  // 将雷达原始数据转换成点云格式
  pcloud_build_by_pos_scan(outscan, pcloud);

  std::cout << "********size*********"<< pcloud.data.size() << std::endl;

  for(auto point : pcloud.data)
  {
    if(point.index > 500)
    {
       std::cout << "********index*********"<< point.index << std::endl;
    }
  }

  // 拟合直线进行畸变处理
  fitting_line(pcloud);

  // 清空带发布的点云
  for (auto &outscan_point : outscan.points)
  {
    memset(&outscan_point, 0, sizeof(LaserPoint));
  }

  for (auto point : pcloud.data)
  {
    if (point.change)
    {
      float distance = sqrt((point.x - pcloud.origin.x) * (point.x - pcloud.origin.x) + (point.y - pcloud.origin.y) * (point.y - pcloud.origin.y));
      outscan.points[point.index].range = distance / 1000.0f;
    }
    else
    {
      outscan.points[point.index].range = point.range;
    }

    outscan.points[point.index].angle = point.angle;
    outscan.points[point.index].intensity = point.intensity;
  }
}
