#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H



enum CONTROL_CMD
{

		Stop=0,           //Car stop
    	TurnRight=1,
		TurnLeft,       //TurnLeft	
		GoAhead,        //Go forward 
		GoBack,         //Go Back 

		//捡球装置
		TurnDownPicker, //TurnDownPicker	
		HandOnPicker,   //HandOnPicker			
		TurnUpPicker,   //TurnUpPicker
		//捡球飞轮
		TurnOnFlyWheel,
		TurnOffFlyWheel,
		//风扇
		TurnOnFan,      //Turn On Wind		
		TurnOffFan      //Turn Off Wind			
		
};


int CarActControl(float &radius,float &angle);

extern int SpeedP,SpeedI,SpeedD; 
extern int ThisAngleBias,LastAngleBias,PreAngleBias;
extern int PwmMotorTmep,PwmMotor;



#endif
