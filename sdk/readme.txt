
Linux:

1. cd sdk

2. mkdir build

3. cd build

4. cmake ..

5. make

6. ./cspc_lidar /dev/sc_mini -version 4

7. 备注:
Linux版本没有标准的数据通讯格式，在main函数的while循环中，会一直接收数据，数据存储在LaserScan结构体中，可根据整体项目情况，使用该数据。

