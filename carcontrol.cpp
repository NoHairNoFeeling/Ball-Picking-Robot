#include "stdio.h"
#include "string.h"
#include "math.h"
#include <unistd.h>
#include "uart.h"
#include "modify.h"
#include "node.h"
#include "carcontrol.h"




//打印调试
#define DEBUG_PROCESS 					0 //打印程序执行到哪一步了
#define DEBUG_CordinationOfBadminton 	0 //用于打印距离最近羽毛球坐标
#define DEBUG_MissUnderstand 			0 //打印 "误判捡球轮为球" 信息

int SpeedP =2,SpeedI =2,SpeedD =2; 
int ThisAngleBias =0,LastAngleBias =0,PreAngleBias =0;
int PwmMotorTmep =0,PwmMotor =0;



int CarActControl(float &radius,float &angle)
{
	float angle_bias,radius_bias;
	bool  Is_A_Ball_In_Middle_Radius=0;

	bool  left_pick_ball_condition=0,right_pick_ball_condition=0;//左右参考依据是雷达的俯视方向
	static bool  IS_BALL_IN_MIDDLE=0;
	
	static int miss_time=0,radius_zero_times=0;
	int speed=0;
	//
	// if(radius==0 || angle==0)
	// 	return 0;
	//Is_Picker_Down_Flag = !(Is_Use_WideRange);
	ThisAngleBias = int(angle  - FRONT_ANGLE);
	
	ThisAngleBias = fabs(ThisAngleBias);
	//angle_bias =angle  - FRONT_ANGLE;
	
	angle_bias =angle  - FRONT_ANGLE;
	radius_bias=radius - FRONT_DISTANCE;
	
	printf("(%4.0f,%3.2f),%3.2f\n",radius,angle,angle_bias);
	//判断是否将捡球轮误认为是羽毛球
	//判断依据是支架下降后捡球轮和激光雷达之间角度和距离关系
	// if( ((angle >= 57.5 && angle<= 72.4)||(angle >= 78.2 && angle<= 93.0)/*角度条件*/) &&\
	// 			(radius >= 260 && radius <= 289)/*距离条件*/ )

	if(Is_Picker_Down_Flag==1)
	{
		//误判左轮为球的情况
		left_pick_ball_condition =(radius <= LEFT_PICKER_MAX_RADIUS && \
								   radius >= LEFT_PICKER_MIN_RADIUS && \
								   angle  >= LEFT_PICKER_START_ANGLE && \
								   angle  <= LEFT_PICKER_END_ANGLE);
		//误判右轮为球的情况						   
		right_pick_ball_condition=(radius >= RIGHT_PICKER_MIN_RADIUS && \
								   radius <= RIGHT_PICKER_MAX_RADIUS && \
								   angle  >= RIGHT_PICKER_START_ANGLE && \
								   angle  <= RIGHT_PICKER_END_ANGLE);
	}

	
	if(left_pick_ball_condition||right_pick_ball_condition)
		{
			Is_Miss_Boll_And_Wheel=1;			
			miss_time++;
			#if DEBUG_MissUnderstand
			printf("misunderstand\n");
			#endif
		}
	else
		{
			Is_Miss_Boll_And_Wheel=0;
			miss_time=0;
		}


	//如果(5圈/S)连续误将捡球轮认为是羽毛球8次,则停车并升起捡球轮支架
	//1秒5圈,也就是1秒误解5次
	#if DEBUG_PROCESS
	printf("miss_time==%d\n",miss_time);
	#endif

	if(miss_time>=5 || radius_zero_times>=5)
	{
		//miss_time>=8指的是将捡球轮误判为羽毛球的次数超过8次,
		//正常情况下,捡球轮支架下降后,雷达只扫描小范围,而不会出现误判捡球轮为羽毛球的情况
		if(Is_Picker_Down_Flag==1)
		{
			SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
			usleep(UpDownDelayTime);//上升\下降期间测量
			Is_Picker_Down_Flag=0;
		}

		SendControlCMDToCarByUart(GoBack,SPEED_STOP+4);		
		
		radius_zero_times=0;
		miss_time=0;	
	}	

	//**********************调整方向*****************************//
	// 调整角度,使得捡球机正对羽毛球//
	// 如果偏移,则进行左转或者右转	
	if( (fabs(angle_bias) > DEVIATION_ANGLE))//使用while?
	{

		//方向偏移,支架升起才调整方向
		if( Is_Picker_Down_Flag==0 )
		{
			IS_BALL_IN_MIDDLE=0;
			
			SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
			//如果球偏右，则车右转
			if(angle_bias < -(DEVIATION_ANGLE) )
				//右转
				{
					//数值转换成字符  
					//sprintf(AngleCharBuf,"%3.3f",point_angle[0]);
					//sprintf(RadiusCharBuf,"%4.2f",point_radius[0]);	
					speed=fabs(angle_bias)/0.35+(SPEED_STOP+80);//speed(5度)==800,speed(120度)==1020
					SendControlCMDToCarByUart(TurnRight,speed);//"TurnRight"
					// //为了解决转向过冲问题,增加延迟时间
					// usleep(angle_bias*10000);
					// SendControlCMDToCarByUart(GoAhead,10);
				//20190415 			printf("TurnRight\n");
				}
			//如果球偏左，则车左转
			if(angle_bias > (DEVIATION_ANGLE) )
				//左转
				{
					speed=fabs(angle_bias)/0.35+(SPEED_STOP+80);//speed(5度)==800,speed(120度)==1020
					SendControlCMDToCarByUart(TurnLeft,speed);
					// //为了解决转向过冲问题,增加延迟时间
					// usleep(angle_bias*10000);
					// SendControlCMDToCarByUart(GoAhead,10);
				//20190415 			printf("TurnLeft\n");			
				}	
		}
		//方向偏移,支架降下时,可能是误判.
		else
		{
			;/* code */
		}			

	}
	//如果球没有偏移，则停止转向
	else
	{
		IS_BALL_IN_MIDDLE=1;
		// SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);	
		// Is_Picker_Down_Flag=0;
		SendControlCMDToCarByUart(GoAhead,SPEED_STOP+4);
		#if DEBUG_PROCESS
		printf("IS_BALL_IN_MIDDLE=1;\n");
		#endif		
		// SendControlCMDToCarByUart(TurnLeft,10);	
		// SendControlCMDToCarByUart(TurnRight,10);			
	}




	// // 调整角度,使得捡球机正对羽毛球(PID方法)
    // // //如果偏移,则进行左转或者右转	
	// // if( (fabs(angle_bias) > DEVIATION_ANGLE) && (Is_Picker_Down_Flag==0) )//使用while?
	// // {
	// // 	IS_BALL_IN_MIDDLE=0;
		
		
	// // 	SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
			////PwmMotorTmep---补偿速度,	ThisAngleBias---本次角度偏差值
			////LastAngleBias---上次角度偏差,PreAngleBias---下次角度偏差
			////SpeedP--比例系数,SpeedI--积分系数,SpeedD--积分系数
	// // 	PwmMotorTmep = \
				(ThisAngleBias - LastAngleBias)*SpeedP + \
				(ThisAngleBias)*SpeedI + \
				(ThisAngleBias + PreAngleBias - 2*LastAngleBias)*SpeedD;
	// // 	//PwmMotor = PwmMotor+PwmMotorTmep;

	// // 	PreAngleBias = LastAngleBias;		
	// // 	LastAngleBias = ThisAngleBias;


	// // 	speed = PwmMotorTmep;
	// // 	printf("speed: %d \n",speed);


	// // 	//如果球偏右，则车右转
	// // 	if(angle_bias < -(DEVIATION_ANGLE) )
	// // 		//右转
	// // 		{
	// // 			//数值转换成字符  
	// // 			//sprintf(AngleCharBuf,"%3.3f",point_angle[0]);
	// // 			//sprintf(RadiusCharBuf,"%4.2f",point_radius[0]);	
	// // 			//speed=fabs(ThisAngleBias)/4.72+16;//speed(2度)==12,speed(105度)==18
	// // 			SendControlCMDToCarByUart(TurnRight,speed);//"TurnRight"
	// // 		//20190415 			printf("TurnRight\n");
	// // 		}
	// // 	//如果球偏左，则车左转
	// // 	if(angle_bias > (DEVIATION_ANGLE) )
	// // 		//左转
	// // 		{
	// // 			//speed=fabs(ThisAngleBias)/4.72+16;//speed(2度)==12,speed(105度)==18
	// // 			SendControlCMDToCarByUart(TurnLeft,speed);
	// // 		//20190415 			printf("TurnLeft\n");			
	// // 		}	
	// // }
	// // //如果球没有偏移，则停止转向
	// // else
	// // {
	// // 	IS_BALL_IN_MIDDLE=1;
	// // 	// SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);	
	// // 	// Is_Picker_Down_Flag=0;
	// // 	// SendControlCMDToCarByUart(GoAhead,10);	
	// // 	SendControlCMDToCarByUart(TurnLeft,10);	
	// // 	SendControlCMDToCarByUart(TurnRight,10);			
	// // }


	//******************行走与捡球*********************//
	//将小车旋转至正对球再执行捡球相关动作
	//球在中间时才捡
	if(IS_BALL_IN_MIDDLE==1 && Is_Miss_Boll_And_Wheel==0)
	{
		//按照距离处理小车速度、升降支架//
		//0-220mm,是球,不可捡,停车、升起支架，以便重新检测羽毛球坐标
		if( (radius>0) && (radius<240))
		{
			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCarByUart(GoAhead,SPEED_STOP+4);
				SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
				usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=0;
				usleep(1000*1000);			
			}

			radius_zero_times=0;
			SendControlCMDToCarByUart(GoBack,22);			
		}
		//测到距离为0，则是羽毛球距离雷达过近，此时应该后退
		else if(radius==0)
		{

			radius_zero_times++;
			//为了避免数据波动带来的误动作,测到距离为0的次数大等于10时再去处理
			if(radius_zero_times>=10)
			{
				//如果距离等于0,并且支架是降下的.则可以判定小车中间没有球,需要升起支架检测
				if(Is_Picker_Down_Flag==1)
				{
					SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);	
					usleep(UpDownDelayTime);//上升\下降期间测量
					Is_Picker_Down_Flag=0;
				}
				//如果距离等于0,并且支架是升起的.则可视范围内没有球,需要旋转找球

				SendControlCMDToCarByUart(GoBack,22);		

			}
		
		}

		//220-260mm,是球是轮不确定,不可捡,停止、升起支架重新检测
		else if(radius>=240 && radius<290)
		{
			//若支架降下,则测到的是球,可捡
			if(Is_Picker_Down_Flag==1)
			{
				//若角度在右轮或者左轮范围,则测到的是捡球轮
				if( Is_Miss_Boll_And_Wheel==1 )
				{
					
					SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
					usleep(UpDownDelayTime);//上升\下降期间测量
					Is_Picker_Down_Flag=0;	

				}
				else
				{
					speed=(radius-240)/0.07692+SPEED_STOP;//speed(240)==719,speed(290)==1369
					SendControlCMDToCarByUart(GoAhead,speed);
					SendControlCMDToCarByUart(TurnDownPicker,SPEED_FULL);
					usleep(UpDownDelayTime);//上升\下降期间测量
					Is_Picker_Down_Flag=1;						
				}
				
			
			}

		}

		//250-320mm,是球是轮不确定,是球可捡,
		//是球的话降下支架后前进,Is_A_Ball_In_Middle_Angle
		//是捡球轮的话需要升起支架
		
		// else if( (radius>=260) && (radius<330))
		// {
		// 	//不是误判
		// 	if( (Is_Use_WideRange==1) && (Is_Miss_Boll_And_Wheel==0)) 
		// 	{
		// 		if(1 /*&& (Is_Picker_Down_Flag==0*/)
		// 		{
		// 			SendControlCMDToCarByUart(TurnDownPicker,SPEED_FULL);
		// 			usleep(2000*1000);//下降支架时间
		// 			Is_Picker_Down_Flag=1;
		// 		}
		// 		speed=(radius-250)/20+14;//speed(250)==14,speed(330)==19
		// 		SendControlCMDToCarByUart(GoAhead,speed);

		// 	}
		// 	//是误判
		// 	else if(Is_Miss_Boll_And_Wheel==1 && miss_time>=8)
		// 	{
		// 		if(1/*Is_Picker_Down_Flag==1*/)
		// 		{
		// 			SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
		// 			//usleep(2000*1000);//下降支架时间
		// 			Is_Picker_Down_Flag=0;
		// 		}
		// 	}
		// 	radius_zero_times=0;
		// }	
		else if( (radius>=290) && (radius<370))
		{
			if(Is_Picker_Down_Flag==0)
			{
				SendControlCMDToCarByUart(TurnDownPicker,SPEED_FULL);
				usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=1;
			}
			speed=(radius-290)/0.3738+1370;//speed(290)==1370,speed(370)==1584
			SendControlCMDToCarByUart(GoAhead,speed);			
		}
		//大于330则往前走
		//330-480mm,是球可捡,下降支架同时前进
		else if(radius>=370 && radius<520 && (Is_Miss_Boll_And_Wheel==0))
		{
			if(Is_Picker_Down_Flag==0)
			{
				SendControlCMDToCarByUart(TurnDownPicker,SPEED_FULL);
				usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=1;
				
			}
			speed=(radius-370)/0.7692+1585;//speed(370)==1585,speed(520)==1780
			SendControlCMDToCarByUart(GoAhead,speed);
			radius_zero_times=0;	
		}
		
		//430-MAX_DISTENCE,是球可捡,前进并升起支架
		else if(radius>=520 && radius<880 &&(Is_Miss_Boll_And_Wheel==0))//该距离下不会出现误判,但为逻辑清楚加上了是否误判
		{

			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
				usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=0;
			}

			speed=(radius-520)/0.1583+1781;//speed(520)==0.2474,speed(880)==0.3  2160				
			SendControlCMDToCarByUart(GoAhead,speed);	
			radius_zero_times=0;	
		}

		//是球可捡,前进并升起支架
		else if(radius>=880)
		{
			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
				usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=0;
			}
			speed=(radius-880)/2.985+2158;//speed(880)==0.3,speed(3000)==0.4  		2880	
			SendControlCMDToCarByUart(GoAhead,speed);
			radius_zero_times=0;	
		}
		
		#if DEBUG_PROCESS
		printf("Is_Use_WideRange ->%d\nIs_Miss_Boll_And_Wheel ->%d\n",Is_Use_WideRange);
		#endif		
	}
	//球不在中间,但是没有误判就要升起支架
	else if( IS_BALL_IN_MIDDLE==0 && Is_Miss_Boll_And_Wheel==0)
	{
		//SendControlCMDToCarByUart(GoAhead,10);
		if(Is_Picker_Down_Flag==1)
		{
			SendControlCMDToCarByUart(TurnUpPicker,SPEED_FULL);
			usleep(UpDownDelayTime);//上升\下降期间测量
			Is_Picker_Down_Flag=0;	
		}	

	}
	//球不在中间,但是误判
	//球在中间,但是误判
	else
	{
		;
	}	
	
	return 0;   
}
