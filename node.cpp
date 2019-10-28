/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"
#include "modify.h"
#include "uart.h"
#include "carcontrol.h"
#include "node.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
#define DEBUG_AllRangePoints 0 //值为1时将打印出360度所有的点
using namespace std;
using namespace everest::hwdrivers;

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        //signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    //uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;

bool Is_Picker_Down_Flag=1;
bool Is_Use_WideRange=1;
bool Is_Miss_Boll_And_Wheel=0;

int main(int argc, char * argv[])
{
	int    opt_com_baudrate = 230400;
    string opt_com_path = "/dev/ttyAL0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;   

    //定义一个用来接收雷达数据的结构体数组,成员有Radius和Angles
    ScanPoints ScanPointsArry[A_CIRCLE];

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        return -1;
    }

    printf("RobotLidar connected\n");

    robotics_lidar.initilize(&serial_connect);

	//recv and Analysis  thread
	bool ret = robotics_lidar.RecvAndAnalysisPthread(&robotics_lidar);
	if(ret)
		printf("RecvAndAnalysisPthread create success!\n");
	else
		printf("RecvAndAnalysisPthread create fail!\n");
    

	TLidarError retvalue;
	
    int FailTimes=5;
  	//start scan：AA 08 00 04 01 01 00 01 B9 00  
	#if 1
	retvalue = robotics_lidar.setLidarWorkMode(HIGHSPEED_SCAN);//
	if(retvalue == EXECUTE_SUCCESS )
    {
        printf("High speed scan set successs!\n");	
    }

	else
    {
        //设定转速失败后,重新再试5次
        while( (FailTimes--) && (retvalue != EXECUTE_SUCCESS) )
        {
            retvalue = robotics_lidar.setLidarWorkMode(HIGHSPEED_SCAN);
		    printf("High speed scan set fail! ...TLidarError=%d...\n",retvalue);
        }

    }

	#endif
	
	//reset lidar：
	#if 0
	retvalue = robotics_lidar.setLidarWorkMode(LIDAR_RESET);
	if(retvalue == EXECUTE_SUCCESS )
		printf("LIDAR reset successs!\n");
	else
		printf("LIDAR reset fail! ...TLidarError=%d...\n",retvalue);
	#endif
	//stop scan: AA 08 00 04 01 01 00 00 B8 00   return: AA 08 00 04 41 01 00 00 F8 00 
	#if 0
	retvalue = robotics_lidar.setLidarWorkMode(IDLE_MODE);
	if(retvalue == EXECUTE_SUCCESS )
		printf("stop scan set successs!\n");
	else
		printf("stop scan set fail! ...TLidarError=%d...\n",retvalue);
	#endif
	//set lidar rotationSpeed: AA 09 00 04 04 02 00 0B 00 C8 00 return:AA 08 00 04 44 01 00 00 FB 00
	#if 0
	retvalue = robotics_lidar.setLidarRotationlSpeed(11);
	if(retvalue == EXECUTE_SUCCESS )
		printf("RotationlSpeed set successs!\n");
	else
		printf("RotationlSpeed set fail! ...TLidarError=%d...\n",retvalue);
	#endif
	
    //开风扇,为简化控制默认开启风扇
    SendControlCMDToCarByUart(TurnOnFan,SPEED_FULL);	
    //开启捡球飞轮,为简化控制过程默认进入程序就开启捡球飞轮
    SendControlCMDToCarByUart(TurnOnFlyWheel,1000);	
    //升高支架
    SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
    usleep(UpDownDelayTime);//上升\下降期间测量
    Is_Picker_Down_Flag=0;
    Is_Use_WideRange=1;

    //放下支架
    // SendControlCMDToCarByUart(TurnDownPicker,95);
    // Is_Picker_Down_Flag=1;
    // Is_Use_WideRange=0;

    //设定车速为0
    SendControlCMDToCarByUart(GoAhead,SPEED_STOP);
	
	int size = 0;
	TLidarGrabResult result;
	CLidarDynamicScan lidar_scan;
	std::vector<RslidarDataComplete> send_lidar_scan_data;
	int lidar_scan_size;
	size_t i;
	RslidarDataComplete one_lidar_data;
   	while (1)
    { 	
		result = robotics_lidar.m_dynamic_scan.getGrabResult();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
				robotics_lidar.m_dynamic_scan.resetGrabResult();
					
                lidar_scan = robotics_lidar.getLidarDynamicScan();
                lidar_scan_size = lidar_scan.getSize();
                send_lidar_scan_data.resize(lidar_scan_size);        
				//one circle: angle and  distance
                for( i = 0; i < lidar_scan_size; i++)
                {            
                    one_lidar_data.angle = lidar_scan.m_angle[i];
                    one_lidar_data.distance = lidar_scan.m_distance[i];
                    send_lidar_scan_data[i] = one_lidar_data;
                    
                    //20190224_将采集到的数据存入数组		   
			        ScanPointsArry[i].Radius=one_lidar_data.distance*1000;
                    ScanPointsArry[i].Angles=one_lidar_data.angle;

                    #if DEBUG_AllRangePoints
                    usleep(200*1000);
                    printf("(%.0f,%.3f)",ScanPointsArry[i].Radius,ScanPointsArry[i].Angles);
                    if(i==6)
                    printf("\n");
                    #endif
                }

				//one circle:Total number of points
                //printf("Lidar count %d!\n", lidar_scan_size);
                #if !(DEBUG_AllRangePoints)
                ModifyPoints( ScanPointsArry,lidar_scan_size );	
                #endif


                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        usleep(10);
       
    }
	
    return 0;
}

/*动态扫描的数据
AA 6D 00 04 14 66 00 E4 03 78 69 00 00 7E 2C 68 2C 7E 2C 86 2C 79 2B 09 2B 8A 2B 73 25 00 00 00 00 42 26 49 26 00 00 1A 27 AD 26 00 00 00 00 00 00 00 00 00 00 00 00 00 00 9B 63 9B 63 F1 65 CC 63 EE 29 3B 2C 18 29 20 29 9F 26 56 26 B4 26 64 19 3F 19 6A 19 BE 19 21 1A 3C 1A 8A 1A BF 1A F9 1A 3A 1B B6 1B 0D 1C AE 1C A7 1D 1C 1F 13 1B 
*/








