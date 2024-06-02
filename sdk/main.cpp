#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include "node_lidar.h"


int parse_args(int argc, char **argv)
{
	struct option opts[] = {
		{"baudrate",1,0,256}, 
		{"version",1,0,257},    
		{0, 0, 0, 0} 
	};
	int ch;
	while ((ch = getopt_long_only(argc, argv, "h", opts, NULL)) != -1)
	{
		switch (ch)
		{
		case 'h':
			printf("lidar node Help:\n");
			printf("./node_lidar /dev/ttyS2 -c -d.\n");
			printf("node lidar -c : no rotate compensate.\n");
			printf("node lidar -f : no lidar data filter.\n");
			printf("node lidar -cover [57,70][101,117][184,196][228,241][291,305][335,350] :note, no blank between []\n");
			exit(0);
			break;

		case 256:
			node_lidar.lidar_general_info.m_SerialBaudrate = atoi(optarg);
			break;

		case 257:
			node_lidar.lidar_general_info.version = atoi(optarg);
			break;
	
		default:
			printf("error: unknow arg in ths args\n");
			break;
		}
	}
	return 0;
}



int main(int argc, char **argv)
{
	/*
	if(argc < 2)
	{
		printf("args error\n");
		return 0;
	}*/
	
	fflush(stdout);

	node_lidar.lidar_general_info.port=argv[1];
	parse_args(argc, argv);

	node_start();

	while(1)
	{
		LaserScan scan;
		if(data_handling(scan))
		{
			//根据自定义需求使用数据
      	}
	}
}                                                                                                           
