#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "carcontrol.h"
#include "node.h"
#include "uart.h"


#define DATAPACK_LENGTH  6//数据包长度

#define DEBUG_Command    1//用于打印控制小车的命令
//打开串口

int open_port(void)

{

	int fd;
		

	fd=open("/dev/ttyAL1",O_RDWR | O_NOCTTY | O_NONBLOCK);
	//printf("fd=%d\n",fd);	

	if(fd==-1)

	{
		//perror("Can't Open SerialPort");

		//printf("fopen error \n");
		usleep(50*1000);//original time is 500*1000
	}

	return fd;
}


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop) 

{ 
     struct termios newtio,oldtio; 
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/ 
    memset(&newtio,0,sizeof(termios));
    memset(&oldtio,0,sizeof(termios));  


    if( tcgetattr( fd,&oldtio)  !=  0) 
    {  
    //perror("SetupSerial 1");
    //printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
    printf("error 2\n");
    usleep(50*1000);//original time is 500*1000
    return -1; 
    } 

    bzero( &newtio, sizeof( newtio ) ); 

/*步骤一，设置字符大小*/ 

    newtio.c_cflag  |=  CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  

/*设置停止位*/ 

    switch( nBits ) 
     { 
     case 7: 
      newtio.c_cflag |= CS7; 
      break; 

     case 8: 
      newtio.c_cflag |= CS8; 
      break; 

     } 

/*设置奇偶校验位*/ 

     switch( nEvent ) 
     { 
     case 'o':
     case 'O': //奇数 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag |= PARODD; 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      break; 

     case 'e':
     case 'E': //偶数 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag &= ~PARODD; 
      break;

	 case 'n':
     case 'N':  //无奇偶校验位 
     newtio.c_cflag &= ~PARENB; 
     break;

     default:
      break;
     } 

     /*设置波特率*/ 

switch( nSpeed ) 
     { 
     case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 

     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 

     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 

     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 

     case 460800: 
      cfsetispeed(&newtio, B460800); 
      cfsetospeed(&newtio, B460800); 
      break; 

     default: 

      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 

     } 

/*设置停止位*/ 

     if( nStop == 1 ) 
      newtio.c_cflag &=  ~CSTOPB; 
     else if ( nStop == 2 ) 
      newtio.c_cflag |=  CSTOPB; 

/*设置等待时间和最小接收字符*/ 

     newtio.c_cc[VTIME]  = 0; 
     newtio.c_cc[VMIN] = 0; 

/*处理未接收字符*/ 
     tcflush(fd,TCIFLUSH); 
/*激活新配置*/ 

if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
     { 
      //perror("com set error"); 
      printf("com set error\n");
      return -1; 
     } 

     //printf("set done!\n"); 
     return 0; 
} 



int SendControlCMDToCarByUart(int command_num,int speed)

{

	int fd;
     int i,count;
	int nSpeed=9600,nBits=8,nStop=1;
	char nEvent='N';
     char *buf,speed_buf[3]={0};

     fd=open_port();
     set_opt(fd,nSpeed, nBits, nEvent, nStop);
	buf=(char *)malloc(sizeof(char)*8);	
	
	i=1;
	
	
	strcpy(buf,"");	  //清空		
	strcpy(buf,"cmd");//帧头
		
	switch(command_num)
	{
		//驱动电机动作
		case Stop:               strcat(buf,"s");break;//Stop all motors  				
		case GoAhead:	          strcat(buf,"f");break;//Go forward 
		case GoBack:	 		strcat(buf,"b");break;//Go Back 
		case TurnRight:		strcat(buf,"r");break;//TurnRight
		case TurnLeft: 		strcat(buf,"l");break;//TurnLeft	

		//捡球支架
		case TurnDownPicker:	strcat(buf,"d");break;//TurnDownPicker	
		case HandOnPicker:	     strcat(buf,"u");break;//HandOnPicker			
		case TurnUpPicker:	 	strcat(buf,"u");break;//TurnUpPicker
          //捡球飞轮
          case TurnOnFlyWheel:     strcat(buf,"p");break;//TurnUpPicker
		case TurnOffFlyWheel:    strcat(buf,"p");break;//TurnUpPicker

		//风扇
		case TurnOnFan:	 	strcat(buf,"w");break;//Turn On Wind		
		case TurnOffFan:	 	strcat(buf,"w");break;//Turn Off Wind	\		
		
		default:
		break;
	}
     //速度为两个字节0000-7199,(A-F不不使用)
     if(speed < SPEED_STOP)
          strcat(buf,"0000");

     else if(speed>=SPEED_STOP && speed<1000) 
     {
          strcat(buf,"0"); 
          sprintf(speed_buf,"%3d",speed);
          strcat(buf,speed_buf);//速度SPEED_STOP - 1000        
     }
      
     else
     {
          sprintf(speed_buf,"%4d",speed);
          strcat(buf,speed_buf);//速度00-99         
     } 

     strcat(buf,"#");//帧尾
     write(fd,buf,10);
     
	
     #if DEBUG_Command
     //usleep(200);
     printf("%s\n",buf);
     #endif
     //usleep(100*1000);	
     memset(buf,0,10);
     free(buf);
     close(fd);	
	return 0;
}

