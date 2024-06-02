#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>
#include "msg_recept.h"
#include "serial_port.h"
#include "lidar_data_processing.h"
#include "mtime.h"
#include "node_lidar.h"
#include <math.h>

bool has_device_header = false;

Lidar_Data_Processing::Lidar_Data_Processing(){
  CheckSumCal = 0;
  CheckSum = 0;	//校验和
  SampleNumlAndCTCal = 0;
  LastSampleAngleCal = 0;
  Valu8Tou16 = 0;
  package_Sample_Index = 0;
  FirstSampleAngle = 0; //< 起始采样角
	LastSampleAngle = 0;  //< 结束采样角
  scan_frequence = 0;	      //协议中雷达转速
  CheckSumResult = false;
  has_package_error = false;
  IntervalSampleAngle = 0.0;
	IntervalSampleAngle_LastPackage = 0.0;
  PackageSampleBytes = 2; //< 一个包包含的激光点数
  package_index = 0;
  recvNodeCount = 0;
  start_t = 0;

  //uint64_t m_node_time_ns = current_times(TIME_NANOSECOND);			  //< time stamp
  //uint64_t m_node_last_time_ns = current_times(TIME_NANOSECOND); //< time stamp
  m_pointTime = 1e9 / 5000;
}

Lidar_Data_Processing::~Lidar_Data_Processing(){
}

/************************************************************************/
/*  向激光雷达发布控制指令　Issue control command to lidar                  */
/************************************************************************/
result_t Lidar_Data_Processing::sendCommand(uint8_t cmd)
{
  uint8_t pkt_header[10];
	
	switch (cmd)
	{
	  case 0x60:
		  break;
	  case 0x65:
		  break;
	  case 0x63:
      pkt_header[0]=0xAA;
      pkt_header[1]=0x55;
      pkt_header[2]=0xF0;
      pkt_header[3]=0x0F;
      node_lidar.serial_port->write_data(pkt_header,4);
		break;
	default:
		break;
	}
  //add by jiang
  return 0;
}

result_t Lidar_Data_Processing::sendData(const uint8_t *data, size_t size)
{

  if (data == NULL || size == 0)
  {
    return RESULT_FAIL;
  }

  size_t r;

  while (size)
  {
    r = node_lidar.serial_port->write_data(data, size);

    if (r < 1)
    {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }
  return RESULT_OK;
}

result_t Lidar_Data_Processing::waitSpeedRight(uint8_t cmd,uint64_t timeout)
{
    int  recvPos     = 0;
    uint32_t startTs = getms();
    uint8_t  recvBuffer[100];
    uint32_t waitTime = 0;
    //uint16_t data_count = 0;
    uint16_t check_sum_cal = 0;  //校验和
    uint16_t data_check_sum = 0; //累加数据
    //uint8_t check_data[4];
    bool head_right = false;

    uint16_t data_lenth = 0; //数据长度

    while ((waitTime = getms() - startTs) <= timeout){
      size_t remainSize = 9;
      size_t recvSize = 0;
      result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);
      if (!IS_OK(ans)) {
        return ans;
      }
      if (recvSize >= remainSize) {
        recvSize = remainSize - recvPos;
      }
      ans = node_lidar.serial_port->read_data(recvBuffer, recvSize);
      if (IS_FAIL(ans)) {
        printf("read waitResponseHeader fail\n");
        return RESULT_FAIL;
      }
      for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvBuffer[pos];
        switch (recvPos) {
          case 0:
            if(currentByte == 0xFA){
              printf("head_speed_000=%x\n",currentByte);
            }else{
              continue;
            }
            break;

          case 1:
            if(currentByte == 0xFA){
              printf("head_speed_111=%x\n",currentByte);
            }else{
              continue;
            }
            break;

          case 2:
            if(currentByte == 0xA5){
              printf("head_speed_222=%x\n",currentByte);
              data_check_sum+=currentByte; 
            }else{
              continue;
            }
            break;
        
          case 3:
            if(currentByte == 0x5A){
              printf("head_speed_333=%x\n",currentByte);
              data_check_sum+=currentByte; 
            }else{
              continue;
            }
            break;

          case 4:
            data_lenth = currentByte;
            data_check_sum+=currentByte; 
            printf("head_speed_444=%x\n",currentByte);
            break; 
          
          case 5:
            data_lenth += (currentByte * 0x100);
            data_check_sum+=currentByte;
            printf("head_speed_555=%x\n",currentByte);
            break;

          case 6:
            check_sum_cal = currentByte;
            printf("head_speed_666=%x\n",currentByte);
            break; 
          
          case 7:
            check_sum_cal += (currentByte * 0x100);
            printf("head_speed_777=%x\n",currentByte);
            break;
          
          case 8:
            if(currentByte==0x01)
            {
              printf("head_speed_888=%x count=%d\n",currentByte,recvPos);
              data_check_sum+=currentByte; 
              head_right = true;
            }
          default:
                break;
          }
        recvPos++;
        if(head_right)
        {
          printf("111\n");
          break;
        }
      }
      printf("recv_pos---=%d\n",recvPos);
      if(recvPos == 9){
        printf("222\n");
        break;
      }
    }

    if(recvPos == 9)
    {
      printf("333\n");
      startTs = getms();
      recvPos = 0;
      while ((waitTime = getms() - startTs) <= timeout)
      {
        size_t remainSize = data_lenth + 1;
        size_t recvSize = 0;
        result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans))
        {
          return ans;
        }
        if(recvSize > remainSize)
        {
          recvSize = remainSize;
        }
        node_lidar.serial_port->read_data(recvBuffer, recvSize);
        for (size_t pos = 0; pos < 20; ++pos)
        {
              data_check_sum += recvBuffer[pos];
        }
        if(check_sum_cal == data_check_sum)
        {
          printf("------TRUE\n");
          return RESULT_OK;
        }else{
          printf("------WRONG\n");
        }
      }
    }
    return RESULT_FAIL;

}


result_t Lidar_Data_Processing::waitResponseHeader(uint8_t cmd, uint64_t timeout)
{
  int  recvPos     = 0;
  uint32_t startTs = getms();
  uint8_t  recvBuffer[10];
  uint32_t waitTime = 0;
  uint8_t data_count = 0;
  uint8_t check_sum_cal = 0;
  uint8_t check_data[4];

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = 4;
    size_t recvSize = 0;
    result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);
    if (!IS_OK(ans)) {
      return ans;
    }
    if (recvSize > remainSize) {
      recvSize = remainSize;
    }
    ans = node_lidar.serial_port->read_data(recvBuffer, recvSize);
    if(node_lidar.lidar_general_info.version == M1C1_Mini_v2 && cmd==0x63)
    {
      check_data[0]=0xA5;
      check_data[1]=0x5A;
      check_data[2]=0xE0;
      check_data[3]=0x08;

    }else{
      check_data[0]=0xAA;
      check_data[1]=0x55;
      check_data[2]=0xFC;
      check_data[3]=0x08;
    }
    if(cmd==0x64)
    {
      check_data[0]=0xA5;
      check_data[1]=0x5A;
      check_data[2]=0x5C;
      check_data[3]=0x07;
    }

    if (IS_FAIL(ans)) {
      printf("read waitResponseHeader fail\n");
      return RESULT_FAIL;
    }
    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = recvBuffer[pos];
      printf("head_byte=%d\n",currentByte);
      switch (recvPos) {
        case 0:
          if(currentByte == check_data[0]){
            check_sum_cal ^= currentByte;
          }else{
            continue;
          }
          break;

        case 1:
          if(currentByte == check_data[1]){
            check_sum_cal ^= currentByte;
          }else{
            continue;
          }
          break;

        case 2:
          if(currentByte == check_data[2]){
            check_sum_cal ^= currentByte;
          }else{
            continue;
          }
          break;
      
        case 3:
          if(currentByte == check_data[3]){
            check_sum_cal ^= currentByte;
            data_count = currentByte;
          }else{
            continue;
          }
          break;

        default:
              break;
        }
      recvPos++;
    }
    if(recvPos == 4){
      break;
    }
  }
  if(recvPos == 4)
  {
    startTs = getms();
    recvPos = 0;
    while ((waitTime = getms() - startTs) <= timeout)
    {
      size_t remainSize = data_count + 1;
      size_t recvSize = 0;
      result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans))
      {
        return ans;
      }
      if(recvSize > remainSize)
      {
        recvSize = remainSize;
      }
      node_lidar.serial_port->read_data(recvBuffer, recvSize);
      for (size_t pos = 0; pos < recvSize - 1; ++pos)
      {
            printf("data*=%d\n",recvBuffer[pos]);
            check_sum_cal ^= recvBuffer[pos];
            if(cmd==0x63)
            {
              node_lidar.recv_encryp[pos] = recvBuffer[pos];
            }else if(cmd==0x64){
              node_lidar.lidar_version[pos] = recvBuffer[pos];
            }
      }
      printf("data**%d,%d,%d\n",check_sum_cal,recvBuffer[remainSize-1],remainSize);
      if(check_sum_cal == recvBuffer[remainSize-1])
      {
        return RESULT_OK;
      }
    }
  }
  return RESULT_FAIL;
}

result_t Lidar_Data_Processing::waitPackage(node_info *node, uint32_t timeout)
{ 
  if(!node_lidar.serial_port)
  {
    return RESULT_FAIL;
  }
  int recvPos = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  uint8_t *packageBuffer = (node_lidar.lidar_general_info.m_intensities) ? 
                                    (uint8_t *)&node_lidar.scan_packages.package.package_Head : (uint8_t *)&node_lidar.scan_packages.packages.package_Head;
  uint8_t package_Sample_Num = 0;
  int32_t AngleCorrectForDistance = 0;
  int package_recvPos = 0;
  uint8_t package_type = 0;

  if (package_Sample_Index == 0)
  {
    recvPos = 0;
    while ((waitTime = getms() - startTs) <= timeout)
    {
      size_t remainSize = PackagePaidBytes - recvPos;
      size_t recvSize = 0;
 
      result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans))
      {
        return ans;
      }

      if (recvSize > remainSize)
      {
        recvSize = remainSize;
      }

      node_lidar.serial_port->read_data(node_lidar.globalRecvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos)
      {
        uint8_t currentByte = node_lidar.globalRecvBuffer[pos];
        switch (recvPos)
        {
        case 0:
          if (currentByte == (PH & 0xFF)){
          }else{
            continue;
          }
          break;

        case 1:
          CheckSumCal = PH;
          if (currentByte == (PH >> 8)){
          }else{
            recvPos = 0;
            continue;
          }
          break;

        case 2:
          SampleNumlAndCTCal = currentByte;
          package_type = currentByte & 0x01;
          if ((package_type == CT_Normal) || (package_type == CT_RingStart))
          {
            if (package_type == CT_RingStart)
            {
              if(node_lidar.lidar_time.tim_scan_start == 0)
              {
                node_lidar.lidar_time.tim_scan_start = getTime();
              }else{
                node_lidar.lidar_time.tim_scan_end = getTime();
              }
              scan_frequence = (currentByte & 0xFE) >> 1;
            }
          }
          else{
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 3:
          SampleNumlAndCTCal += (currentByte * 0x100);
          package_Sample_Num = currentByte;
          break;

        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
          {
            FirstSampleAngle = currentByte;
          }else{
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 5:
          FirstSampleAngle += currentByte * 0x100;
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle >> 1;
          break;

        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
          {
            LastSampleAngle = currentByte;
          }
          else
          {
            has_package_error = true;
            recvPos = 0;
            continue;
          }
          break;

        case 7:
          LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle >> 1;

          if (package_Sample_Num == 1)
          {
            IntervalSampleAngle = 0;
          }
          else{
            if (LastSampleAngle < FirstSampleAngle)
            {
              if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64))
              {
                IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle - FirstSampleAngle) /
                                                          ((package_Sample_Num - 1) *1.0));
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              }
              else
              {
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            }
            else
            {
              IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) /
                                                          ((package_Sample_Num - 1) * 1.0));
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }

          break;

        case 8:
          CheckSum = currentByte;
          break;

        case 9:
          CheckSum += (currentByte * 0x100);
          break;
        }
        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos == PackagePaidBytes)
      {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos)
    {
      startTs = getms();
      recvPos = 0;

      while ((waitTime = getms() - startTs) <= timeout)
      {
        size_t remainSize = package_Sample_Num * PackageSampleBytes - recvPos;
        size_t recvSize = 0;
        result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans))
        {
          return ans;
        }

        if (recvSize > remainSize)
        {
          recvSize = remainSize;
        }

        node_lidar.serial_port->read_data(node_lidar.globalRecvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos)
        {
            if (node_lidar.lidar_general_info.m_intensities) {
              if (recvPos % 3 == 2) {
                Valu8Tou16 += node_lidar.globalRecvBuffer[pos] * 0x100;
                CheckSumCal ^= Valu8Tou16;
              } else if (recvPos % 3 == 1) {
                Valu8Tou16 = node_lidar.globalRecvBuffer[pos];
              } else {
                Valu8Tou16 = node_lidar.globalRecvBuffer[pos];
                Valu8Tou16 += 0x00 * 0x100;
                CheckSumCal ^= node_lidar.globalRecvBuffer[pos];
              }
            } else {
              if (recvPos % 2 == 1) {
                Valu8Tou16 += node_lidar.globalRecvBuffer[pos] * 0x100;
                CheckSumCal ^= Valu8Tou16;
              } else {
                Valu8Tou16 = node_lidar.globalRecvBuffer[pos];
              }
            }
          packageBuffer[package_recvPos + recvPos] = node_lidar.globalRecvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * PackageSampleBytes == recvPos)
        {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_Sample_Num * PackageSampleBytes != recvPos)
      {
        return RESULT_FAIL;
      }
    }else{
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;
    if (CheckSumCal != CheckSum)
    {
      printf("data check,%x,%x,%d,%d\n",CheckSumCal,CheckSum,package_Sample_Num,recvPos);
      CheckSumResult = false;
      has_package_error = true;
    }
    else
    {
      CheckSumResult = true;
    }
  }
  uint8_t package_CT;
  if(node_lidar.lidar_general_info.m_intensities)
  {
    package_CT = node_lidar.scan_packages.package.package_CT;
  }else{
    package_CT = node_lidar.scan_packages.packages.package_CT;
  }

  (*node).scan_frequence = 0;

  if ((package_CT & 0x01) == CT_Normal)
  {
    (*node).sync_flag = Node_NotSync;
    memset((*node).debug_info, 0xff, sizeof((*node).debug_info));

    if (!has_package_error)
    {
      if (package_index < 10)
      {
        (*node).debug_info[package_index] = (package_CT >> 1);
        (*node).index = package_index;
      }else{
        (*node).index = 0xff;
      }
      if (package_Sample_Index == 0)
      {
        package_index++;
      }
    }else{
      (*node).index = 255;
      package_index = 0;
    }
  }else{
    (*node).sync_flag = Node_Sync;
    (*node).index = 255;
    package_index = 0;

    if (CheckSumResult)
    {
      has_package_error = false;
      (*node).scan_frequence = scan_frequence;
    }
  }

  (*node).sync_quality = Node_Default_Quality;
  (*node).stamp = 0;

  if (CheckSumResult)
  {
    if(node_lidar.lidar_general_info.m_intensities)
    {
      
      (*node).distance_q2 = (node_lidar.scan_packages.package.packageSampleDistance[package_Sample_Index*3+2] * 64) + 
                            (node_lidar.scan_packages.package.packageSampleDistance[package_Sample_Index*3+1] >> 2);
      (*node).sync_quality = (node_lidar.scan_packages.package.packageSampleDistance[package_Sample_Index*3+1] & 0x03)*64 + 
                            (node_lidar.scan_packages.package.packageSampleDistance[package_Sample_Index*3] >> 2);
      (*node).exp_m = node_lidar.scan_packages.package.packageSampleDistance[package_Sample_Index*3] & 0x01;
      //(*node).distance_q2 = (node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index*3+1] >> 2 | (node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index*3+2] << 6));
      //(*node).sync_quality = ((node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index*3] >> 2) & 0x3F) + (node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index*3+1] & 0x03) * 64;
    }else{
      (*node).distance_q2 = node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index]>>2;
      (*node).sync_quality = ((uint16_t)((node_lidar.scan_packages.packages.packageSampleDistance[package_Sample_Index]) & 0x03));
    }
    
    if ((*node).distance_q2 != 0)
    {
      //结构引起的补偿
      /*
      AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((*node).distance_q2 / 4.0))) / 155.3) /
                                                 ((*node).distance_q2 / 4.0))) *180.0 / 3.1415) * 64.0);*/

      AngleCorrectForDistance = (int32_t)(atan(19.16*((*node).distance_q2-90.15)/(90.15*(*node).distance_q2))*64);
      
    }else{
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;

    if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0)
    {
      (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                               AngleCorrectForDistance + 23040))
                                   << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                  LIDAR_RESP_MEASUREMENT_CHECKBIT;
    }else{
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040)
      {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                                 AngleCorrectForDistance - 23040))
                                     << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }else{
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                                 AngleCorrectForDistance))
                                     << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  }else{
    (*node).sync_flag = Node_NotSync;
    (*node).sync_quality = Node_Default_Quality;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q2 = 0;
    (*node).scan_frequence = 0;
  }

  uint8_t nowPackageNum;

  if(node_lidar.lidar_general_info.m_intensities)
  {
    nowPackageNum = node_lidar.scan_packages.package.nowPackageNum;
  }else{
    nowPackageNum = node_lidar.scan_packages.packages.nowPackageNum;
  }

  package_Sample_Index ++;

  if (package_Sample_Index >= nowPackageNum)
  {
    package_Sample_Index = 0;
    CheckSumResult = false;
  }
  return RESULT_OK;
}


result_t Lidar_Data_Processing::waitPackage_coin(node_info *node,uint32_t timeout)
{
  
  int recvPos = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;

  uint8_t *packageBuffer = (uint8_t *)&node_lidar.scan_packages.package_coin.headL;
  uint8_t package_Sample_Num = 0;
  int32_t AngleCorrectForDistance = 0;
  int package_recvPos = 0;
  uint8_t package_type = 0;

  uint16_t start;
  uint16_t stop;
  float sampleAngle_t = 0.0;


  if (package_Sample_Index == 0)
  {
    recvPos = 0;

    while ((waitTime = getms() - startTs) <= timeout)
    {
      size_t remainSize = PackagePaidBytes - recvPos;
      size_t recvSize = 0;
      result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans))
      {
        return ans;
      }

      if (recvSize > remainSize)
      {
        recvSize = remainSize;
      }

      node_lidar.serial_port->read_data(node_lidar.globalRecvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos)
      {
        uint8_t currentByte = node_lidar.globalRecvBuffer[pos];
        switch (recvPos)
        {
        case 0:
          if (currentByte == 0x55)
          {
            //printf("收到头数据帧\n");
          }
          else{
            continue;
          }
          break;
        case 1:
          if (currentByte == 0xAA){
          }
          else{
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          if(currentByte==0x83){
          }
          else{
            recvPos = 0;
            continue;
          }
          break;

        case 3:
          if(currentByte==0x08){ 
            package_Sample_Num = currentByte;           
          }
          else{
            recvPos = 0;
            continue;
          }
          break;

        case 4:
          break;

        case 5:
          break;

        case 6:
          FirstSampleAngle = currentByte;
          break;
        
        case 7:
          FirstSampleAngle += currentByte * 0x100;
          
          start_t = 360.0F * (FirstSampleAngle - FRAME_ANGLE_BASE)/FRAME_ANGLE_RANGE;
          //node_lidar.FirstSampleAngle = node_lidar.FirstSampleAngle >> 1;
          //node_lidar.FirstSampleAngle = node_lidar.FirstSampleAngle >> 1;
          break;
        }    

        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos == PackagePaidBytes)
      {
        package_recvPos = recvPos;
        break;
      }
    }
    int k;
    if (PackagePaidBytes == recvPos)
    {
      startTs = getms();
      recvPos = 0;
    
      while ((waitTime = getms() - startTs) <= timeout)
      {
        size_t remainSize = package_Sample_Num * PackageSampleBytes + 4 - recvPos;
        size_t recvSize = 0;
        result_t ans = node_lidar.serial_port->waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans))
        {
          return ans;
        }
        if (recvSize > remainSize)
        {
          recvSize = remainSize;
        }

        node_lidar.serial_port->read_data(node_lidar.globalRecvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
          packageBuffer[package_recvPos + recvPos] = node_lidar.globalRecvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * PackageSampleBytes + 4 == recvPos)
        {
          package_recvPos += recvPos;
          break;
        }
      }
      if (package_Sample_Num * PackageSampleBytes + 4 != recvPos)
      {
        return RESULT_FAIL;
      }
    }
    else
    {
      return RESULT_FAIL;
    }
    unsigned int datajgs[17]={0};
    int temp_i=0;
    unsigned int chk32 = 0;
    for(temp_i = 0;temp_i < 17;temp_i++)
    {
      datajgs[temp_i]=packageBuffer[2*temp_i]+(packageBuffer[2*temp_i+1]<<8);
    }

    for(temp_i =0;temp_i<17;temp_i++)
    {
      chk32 =(chk32 <<1)+datajgs[temp_i];
    }
    CheckSumCal = (chk32 & 0x7FFF)+(chk32 >>15);
    CheckSumCal = CheckSumCal &0x7FFF;
    //node_lidar.LastSampleAngle = 360.0F * ((node_lidar.scan_packages.package_coin.stopAngleL + node_lidar.scan_packages.package_coin.stopAngleH*256)-FRAME_ANGLE_BASE)/FRAME_ANGLE_RANGE;
    LastSampleAngle = node_lidar.scan_packages.package_coin.stopAngleL;
    LastSampleAngle += node_lidar.scan_packages.package_coin.stopAngleH * 0x100;
    
    stop_t = 360.0F * (LastSampleAngle - FRAME_ANGLE_BASE)/FRAME_ANGLE_RANGE;
    //printf("node_lidar.LastSampleAngle =%f\n",360.0F * (node_lidar.LastSampleAngle - FRAME_ANGLE_BASE)/FRAME_ANGLE_RANGE);
    
    //node_lidar.LastSampleAngle = node_lidar.LastSampleAngle >> 1;
    
    /*
    if (node_lidar.LastSampleAngle < node_lidar.FirstSampleAngle) {
      if ((node_lidar.FirstSampleAngle > 270 * 64) && (node_lidar.LastSampleAngle < 90 * 64)) {
        node_lidar.IntervalSampleAngle = (float)((360 * 64 + node_lidar.LastSampleAngle -
                                        node_lidar.FirstSampleAngle) / ((
                                              package_Sample_Num - 1) * 1.0));
        node_lidar.IntervalSampleAngle_LastPackage = node_lidar.IntervalSampleAngle;
      } else {
        node_lidar.IntervalSampleAngle = node_lidar.IntervalSampleAngle_LastPackage;
      }
    } else {
      node_lidar.IntervalSampleAngle = (float)((node_lidar.LastSampleAngle - node_lidar.FirstSampleAngle) / ((
                                      package_Sample_Num - 1) * 1.0));
      node_lidar.IntervalSampleAngle_LastPackage = node_lidar.IntervalSampleAngle;
    }*/
    if(LastSampleAngle < FirstSampleAngle)
    {
      IntervalSampleAngle = (360 - start_t + stop_t)/(package_Sample_Num-1);

    }
    else
    {
      IntervalSampleAngle = (stop_t - start_t)/(package_Sample_Num-1);

    }
    if (node_lidar.scan_packages.package_coin.csL != (CheckSumCal & 0xFF) || node_lidar.scan_packages.package_coin.csH != (CheckSumCal >> 8))
    {
      CheckSumResult = false;
      has_package_error = true;
    }
    else
    {
      CheckSumResult = true;
    }
  }
  /*
  uint8_t package_CT;

  if (node_lidar.m_intensities)
  {
    package_CT = node_lidar.scan_packages.package.package_CT;
  }
  else
  {
    package_CT = node_lidar.scan_packages.packages.package_CT;
  }

  (*node).scan_frequence = 0;

  if ((package_CT & 0x01) == CT_Normal)
  {
    (*node).sync_flag = Node_NotSync;
    memset((*node).debug_info, 0xff, sizeof((*node).debug_info));

    if (!node_lidar.has_package_error)
    {
      if (node_lidar.scan_packages.package_index < 10)
      {
        (*node).debug_info[node_lidar.scan_packages.package_index] = (package_CT >> 1);
        (*node).index = node_lidar.scan_packages.package_index;
      }
      else
      {
        (*node).index = 0xff;
      }
      if (node_lidar.scan_packages.package_Sample_Index == 0)
      {
        node_lidar.scan_packages.package_index++;
      }
    }
    else
    {
      (*node).index = 255;
      node_lidar.scan_packages.package_index = 0;
    }
  }
  else
  {
    (*node).sync_flag = Node_Sync;
    (*node).index = 255;
    node_lidar.scan_packages.package_index = 0;

    if (node_lidar.CheckSumResult)
    {
      node_lidar.has_package_error = false;
      (*node).scan_frequence = node_lidar.scan_frequence;
    }
  }*/

  (*node).sync_quality = Node_Default_Quality;
  (*node).stamp = 0;

  if (CheckSumResult)
  {
    (*node).distance_q2 = (node_lidar.scan_packages.package_coin.data[package_Sample_Index].d0 +
          (node_lidar.scan_packages.package_coin.data[package_Sample_Index].d1*256))&0x3FFF;
   
    (*node).sync_quality = (node_lidar.scan_packages.package_coin.data[package_Sample_Index].d2>>2);
    

    if ((*node).distance_q2 != 0)
    {
      AngleCorrectForDistance = 0; 
    }
    else
    {
      AngleCorrectForDistance = 0;
    }

    sampleAngle_t = IntervalSampleAngle * package_Sample_Index + start_t;
    if(sampleAngle_t >360)
    {
      sampleAngle_t = sampleAngle_t - 360;
    }
    (*node).angle_q6_checkbit = (uint16_t)(sampleAngle_t*100); 
  }
  
  else
  {
    (*node).sync_flag = Node_NotSync;
    (*node).sync_quality = Node_Default_Quality;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q2 = 0;
    (*node).scan_frequence = 0;
  }
  angle_new = sampleAngle_t;
  if(angle_bak >350 && angle_new <10)
  {
    (*node).sync_flag = Node_Sync;
  }
  else
  {
    (*node).sync_flag = Node_NotSync;
  }
  
  angle_bak = angle_new;
					
  uint8_t nowPackageNum;
  nowPackageNum = node_lidar.scan_packages.package_coin.sampleinfo;

  package_Sample_Index++;

  if (package_Sample_Index >= nowPackageNum)
  {
    package_Sample_Index = 0;
    CheckSumResult = false;
  }
  return RESULT_OK;
}


result_t Lidar_Data_Processing::waitScanData(node_info *nodebuffer, size_t &count, uint32_t timeout)
{
  if(!node_lidar.lidar_status.isConnected)
  {
    count = 0;
    return RESULT_FAIL;
  }

  recvNodeCount = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  result_t ans = RESULT_FAIL;
  /*超时处理及点数判断*/
  while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count)
  {
    node_info node;
    if(node_lidar.lidar_general_info.version == M1CT_Coin_Plus)
    {
      ans = waitPackage_coin(&node, timeout - waitTime);
    }else{
      ans = waitPackage(&node, timeout - waitTime);
    }
    
    if (!IS_OK(ans))
    {
      count = recvNodeCount;
      return ans;
    }

    nodebuffer[recvNodeCount++] = node;

    if (node.sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
    {
      /*检查缓冲区中的字节数*/
      size_t size = node_lidar.serial_port->available();
      uint64_t delayTime = 0;
      /*数据包的数据量*/
      size_t PackageSize = (node_lidar.lidar_general_info.m_intensities ? INTENSITY_NORMAL_PACKAGE_SIZE :
                            NORMAL_PACKAGE_SIZE);

      if (size > PackagePaidBytes && size < PackagePaidBytes * PackageSize)
      {
        size_t packageNum = size / PackageSize;
        size_t Number = size % PackageSize;
        delayTime = packageNum * node_lidar.lidar_general_info.m_PointTime * PackageSize / 2;

        if (Number > PackagePaidBytes)
        {
          delayTime += node_lidar.lidar_general_info.m_PointTime * ((Number - PackagePaidBytes) / 2);
        }

        size = Number;

        if (packageNum > 0 && Number == 0)
        {
          size = PackageSize;
        }
      }
      nodebuffer[recvNodeCount - 1].stamp = size * node_lidar.lidar_general_info.trans_delay + delayTime;
      nodebuffer[recvNodeCount - 1].scan_frequence = node.scan_frequence;
      count = recvNodeCount;
      return RESULT_OK;
    }
    if (recvNodeCount == count)
    {
      return RESULT_OK;
    }
  }
  count = recvNodeCount;
  return RESULT_FAIL;
}

