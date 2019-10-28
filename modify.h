#ifndef __MODIFY_H
#define __MODIFY_H

#include "node.h"

#define FRONT_ANGLE      75.74 	//定义正前方角度
#define DEVIATION_ANGLE  4  	//定义角度偏差为2度，球在角度偏差范围内都能捡到
#define FRONT_DISTANCE   300	//该距离下捡球轮恰好能捡到球
#define DEVIATION_RADIUS 50 	//考虑测量误差

#define CHANGE_GRAD 			70		//两个点之间最大变化值为70mm
#define LIDAR_SCAN_START_ANGLE 	30.0	//limit  20.0,捡球飞轮升起后可视范围起始角
#define LIDAR_SCAN_END_ANGLE   	120.0	//limit 130.0,捡球飞轮升起后可视范围终止角
#define MAX_RADIUS  			2000	//最大量程3500
#define MAX_SHUTTLECOCK 		50		//可视范围最多有n个球

#define MIDDLE_PICK_START_ANGLE 71.90	//limit74.38,捡球飞轮降下后可视范围起始角
#define MIDDLE_PICK_END_ANGLE 	79.58	//limit79.89,捡球飞轮降下后可视范围终止角

#define LEFT_PICKER_START_ANGLE 61.35  	//雷达测到的左边捡球飞轮的起始角度
#define LEFT_PICKER_END_ANGLE   71.90  	//雷达测到的左边捡球飞轮的终止角度
#define LEFT_PICKER_MIN_RADIUS  290     //左边捡球轮距离最小值
#define LEFT_PICKER_MAX_RADIUS  317     //左边捡球轮距离最大值

#define RIGHT_PICKER_START_ANGLE 79.58  //雷达测到的左边捡球飞轮的起始角度
#define RIGHT_PICKER_END_ANGLE  91.2 	//雷达测到的左边捡球飞轮的终止角度
#define RIGHT_PICKER_MIN_RADIUS 281     //右边捡球轮距离最小值
#define RIGHT_PICKER_MAX_RADIUS 302     //右边捡球轮距离最大值

#define MIN_LENGTH_WHEEL		20//雷达测到的左边捡球飞轮的最小长度
#define MAX_LENGTH_WHEEL		90//雷达测到的左边捡球飞轮的最达长度

#define MAXLENTH 100.00         //羽毛球最大长度
#define MINLENTH 50.00          //羽毛球最小长度

int ModifyPoints(ScanPoints (&StructArry)[A_CIRCLE],int count);

#endif
