#ifndef __NODE_H
#define __NODE_H

#define MAXPOINT 8000
#define A_CIRCLE 1600  //高速扫描模式，转速为5r/s，每圈的点数为1600，实际情况一般为1520个点

#define SPEED_STOP    719  //电机的停止速度
#define SPEED_FULL   7199  //电机电机全速
#define UpDownDelayTime 2000*1000

struct ScanPoints
{
    float Radius;
    float Angles;
};

extern bool Is_Picker_Down_Flag;
extern bool Is_Use_WideRange;
extern bool Is_Miss_Boll_And_Wheel;

extern ScanPoints ScanPointsArry[A_CIRCLE];

#endif
