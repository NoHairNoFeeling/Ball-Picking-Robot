#include "stdio.h"
#include "stdlib.h"
#include <unistd.h>
#include <math.h>
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"
#include <cstdlib>

#include "uart.h"
#include "carcontrol.h"
#include "node.h"
#include "modify.h" 




#define DEBUG_ACirclePoints		0//DEBUG_ACirclePoints为真时函数把雷达可测点的坐标打印出来
#define DEBUG_PROCESS 			0//用于打印程序执行到哪儿了
#define DEBUG_ScanRange 		0//用于打印 扫描模式(大范围还是小范围扫描)
#define DEBUG_UsingSmallRange 	1//为1时用于打印小范围测量时  所有测到的点
#define DEBUG_Points			1//为0时程序打印点,不做动作.使用时 令 DEBUG_ACirclePoints=1

using namespace std;
using namespace everest::hwdrivers;
/*
本程序功能是找出可视范围内的羽毛球可疑物,并将最近可疑物的坐标传给CarActControl()
步骤:s
1.筛选出可视范围的点
2.找出可视范围所有可疑点
3.找出可疑点中离最近的传给CarActControl()
4.判断捡球飞轮中间是否有球
*/


int ModifyPoints(ScanPoints (&StructArry)[A_CIRCLE],int count)
{
	int   i,j,k=0,m=0,m_l=0;
	int   TempCount,SwapAngle,SwapRadius;	
	float angle_resolution,radius_resolution;
	static float angle_average=0,radius_average=0;
	static float start_angle,end_angle;
	static int   point_num_min=0,point_num_max=0;//point_num:覆盖一个球需要的点数，zero_count：允许数据波动为零的点数
	float point_radius[MAX_SHUTTLECOCK]={0},point_angle[MAX_SHUTTLECOCK]={0};   //放置计算好的羽毛球位置坐标
	
	static int	 PickerDown_and_RadiusZeroTimes=0;//支架降下后,测到距离为零的次数
	// char  *AngleCharBuf,*RadiusCharBuf;

	
    if( StructArry==NULL || count==0 )
	{
        //    printf("传值错误！\n");	
           return -1;
	}

    angle_resolution  =   360.00/count; 	//Angle resolution
    radius_resolution = 2*3.1415/count;		//Radian resolution		
    
	//先确定采用大范围扫描还是小范围扫描
	//依据:捡球轮支架降下后采用小范围检测,其他情况采用大范围扫描
	//捡球轮降下的表现:
	//1.角度在[LEFT_PICK_START_ANGLE,LEFT_PICK_START_ANGLE]范围内测出一系列距离为[272,276]的点
	//2.角度在[RIGHT_PICKER_START_ANGLE,RIGHT_PICKER_END_ANGLE]范围内测出一系列距离为[272,299]的点
	//3.程序中所有的左捡球轮和右捡球轮都是以激光雷达的俯视图为参考的，而激光雷达和小车的俯视图正好反向
	//也就是说程序中的左右捡球轮对应实际俯视小车的右左捡球轮

	//利用支架的下降与否确定采用大范围还是小范围扫描
	//如果支架下降标志Is_Picker_Down_Flag==1,则启用小范围扫描
	if(Is_Picker_Down_Flag==1)
	{
		Is_Use_WideRange=0;	
		start_angle= MIDDLE_PICK_START_ANGLE;
		end_angle  = MIDDLE_PICK_END_ANGLE;
		#if DEBUG_ScanRange 

		printf("\n");
		printf("using small range \n");
		printf("start:  %.2f  , end:  %.2f  \n",start_angle,end_angle);
		#endif
	}
	else
	{
		Is_Use_WideRange=1;	
		start_angle	= LIDAR_SCAN_START_ANGLE;
		end_angle	= LIDAR_SCAN_END_ANGLE;
		#if DEBUG_ScanRange 	
		printf("\n");	
		printf("using wide range \n");
		printf("start:  %.2f  , end:  %.2f  \n",start_angle,end_angle);		
		#endif				
	}
	

	#if DEBUG_PROCESS 
	printf("Use the ball to determine whether to enable small range detection\n");
	#endif	

	//检测羽毛球坐标步骤：
	//1.判断是否为羽毛球；半径变化和角度跨度变化 满足羽毛球的外形特征
	//2.计算羽毛球坐标:求出满足条件1的所有点坐标平均值
	
	for(i=(int)(count*start_angle/360.0);i<=(int)(count*end_angle/360.0)+1;i++)
	  {
		//5个一组,不区分大范围或是小范围,打印测量的所有点坐标信息
		#if DEBUG_ACirclePoints  				
		printf("(%4.0f,%3.1f) ",StructArry[i].Radius,StructArry[i].Angles);
		if(i%5==0)
		printf("\n");
		usleep(200*1000);
		#endif
#if DEBUG_Points
 		//限定距离为MAX_RADIUS，超过该距离点数太少不精确 
		//由于超过 MAX_RADIUS 距离的点会全部显示为0 ,所以程序中应该把0也排除
		if( StructArry[i].Radius <= MAX_RADIUS && StructArry[i].Radius > 0)	
		{
			//计算该距离覆盖一个羽毛球需要的最小点数
			//支架降下时,羽毛球的点数按照实验数据给出
			if(Is_Picker_Down_Flag==1)
				point_num_min=		16/(StructArry[i].Radius)/radius_resolution;
			else
				point_num_min=MINLENTH/(StructArry[i].Radius)/radius_resolution;

			//计算该距离覆盖一个羽毛球需要的最大点数
			point_num_max=MAXLENTH/(StructArry[i].Radius)/radius_resolution;

			TempCount=0;
			//统计距离变化不超过CHANGE_GRAD的点,
			j=i;

			//只要连续点半径变化不大于CHANGE_GRAD就认为该物体是连续的
			//连续的点数不能太多,也不能太少,否则不是羽毛球
		    while( (fabs( StructArry[j].Radius-StructArry[i].Radius ) <= CHANGE_GRAD) && TempCount<=point_num_max+10)
			    {
					TempCount++; 
					j++;
					#if DEBUG_PROCESS 
					printf("fabs:%f\n",fabs( StructArry[j].Radius-StructArry[i].Radius ));
					printf("%2d,StructArry->Radius[%d] == %f \n",TempCount,j,StructArry[j].Radius);
					#endif
			    }	
	  
			//通过点数粗判该物体是否为羽毛球。若是，则求出该物体坐标 
			//判断依据：将羽毛球假想成矩形物体，则羽毛球是一个
			//弧度连续(point_num_min<=TempCount<=point_num_max)、
			//表面平滑的物体(相邻点径向距离差值小于CHANGE_GRAD)

			//point_num_max+1和point_num_min-1中的+1和-1是为了消除程序对(float->int )转换中误差引起的值偏小或者偏大
            if(TempCount<=point_num_max+1 && TempCount>=point_num_min-1)	
		        {
					
					angle_average =0;  
					radius_average=0; 
					for(j=i;j<i+TempCount;j++)			
						{ 
							//printf("%d|%d, angles=%.4f, radius=%.4f\n", j,TempCount, StructArry->Angles[j], StructArry->Radius[j]);
							angle_average +=StructArry[j].Angles;				
							radius_average+=StructArry[j].Radius;	
							// #if DEBUG_UsingSmallRange 
							// 	printf("number=%d,radius=%.2f,angle=%.3f\n",j,StructArry[j].Radius,StructArry[j].Angles);
							// 	printf("number=%d,radius_average=%.2f,angle_average=%.3f\n",j-i,radius_average,angle_average);
							// #endif				
						}	
			
			        angle_average /=(TempCount);	
			        radius_average/=(TempCount);

					#if DEBUG_PROCESS 
			        printf("radius_average=%.4f,angle_average=%.3f\n", radius_average,angle_average);
					#endif	

					// #if DEBUG_UsingSmallRange 
			        // 	printf("\n**TempCount=%d**radius_average=%.2f,angle_average=%.3f**********\n",TempCount,radius_average,angle_average);
					// #endif		
			        
					//冗余纠错:求出的平平均值必须在扫描视角范围内
			        if( angle_average >= start_angle && angle_average <= end_angle )
			        {			        
						point_radius[k]=radius_average;
						point_angle[k] = angle_average;				    

						#if DEBUG_PROCESS 
						printf("羽毛球可疑点:radius[%d]=%4.4fmm,angle[%d]=%4.4fdegree\n",k,point_radius[k],k,point_angle[k]);
						#endif
				
						k++;					
					
		            }	
		        }
      	}
	  
		i=i+TempCount;//跳过连续点,  
#endif	
    }
#if DEBUG_Points	
	#if DEBUG_PROCESS 
	printf("Steps to detect badminton coordinates:\n");
	#endif
    //20190415 	printf("\nbefore sort number point_radius[0]=%4.2f,point_angle[0]=%3.3f\n",point_radius[0],point_angle[0]);	
	
	//计算距离最近的羽毛球，因为从路径规划的角度来看捡最近的比较合理
	for(i=0;i<k;i++)
	{
		if(point_radius[0] > point_radius[i] && point_radius[i]>0)
		{
			SwapRadius=point_radius[i];			
			point_radius[i]=point_radius[0];
			point_radius[0]=SwapRadius;	

			SwapAngle=point_angle[i];
			point_angle[i]=point_angle[0];
			point_angle[0]=SwapAngle;
		}
	}

	#if DEBUG_PROCESS 
	printf("calculate the close-in ball\n");
	#endif

	#if DEBUG_ScanRange
   	printf("--------->>after sort %4.1f,%3.1f\n",point_radius[0],point_angle[0]);
	printf("--------->>Is_Picker_Down_Flag==%d\n",Is_Picker_Down_Flag);  
	#endif


	//打印小范围扫描到的所有点
	#if DEBUG_UsingSmallRange	
	if(Is_Picker_Down_Flag==1)
	{
		for(m_l=0,m=(int)(count*start_angle/360.0);m<=(int)(count*end_angle/360.0);m++,m_l++)
		{
			// printf("(%4.0f,%3.2f)",StructArry[m].Radius,StructArry[m].Angles);
			// if(m_l%6==0)
			// printf("\n");
			;
		}
	}
	#endif
	


	if(Is_Picker_Down_Flag==1)
	{
		if( (point_angle[0]>= MIDDLE_PICK_START_ANGLE-7) && \
			(point_angle[0]<= MIDDLE_PICK_END_ANGLE  +7) && \
			 point_radius[0]!=0 && Is_Miss_Boll_And_Wheel==0) 
			{
				CarActControl(point_radius[0],point_angle[0]);
				PickerDown_and_RadiusZeroTimes=0;
			}

		else
		{
			PickerDown_and_RadiusZeroTimes++;
		}

		if(PickerDown_and_RadiusZeroTimes >=5)	
		{
			SendControlCMDToCarByUart(GoAhead,SPEED_STOP+4);			
			SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
			usleep(UpDownDelayTime);//上升\下降期间测量
			Is_Picker_Down_Flag=0;
		}
		#if DEBUG_ScanRange
		printf("--------->>after sort %4.1f,%3.1f\n",point_radius[0],point_angle[0]);
		printf("\n"); 
		#endif
	}
	else
	{
		if(point_radius[0] != 0 ) //&& Is_Miss_Boll_And_Wheel ==0
			CarActControl(point_radius[0],point_angle[0]);	//通过坐标获取小车控制信息[方向]
	}

#endif

	#if DEBUG_PROCESS 
	printf("point_radius[0]=%4.0f,point_angle[0]=%4.0f,Is_Use_WideRange=%d\n",point_radius[0],point_angle[0],Is_Use_WideRange);
	#endif
	//数值转换成字符  
	//sprintf(AngleCharBuf,"%3.3f",point_angle[0]);
	//sprintf(RadiusCharBuf,"%4.2f",point_radius[0]);	
	
	// if(StructArry!=0)
	// {
	// 	delete(StructArry);
	// 	StructArry=NULL;
	// }
		

    return 0;
}
