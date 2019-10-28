#ifndef __UART_H
#define __UART_H




int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop) ;
int open_port(void);
int SendPositionToCarByUart(char *send_radius,char *send_angles);
int SendControlCMDToCarByUart(int command_num,int speed);
//int SendToCar(int argc, char * argv[]);
#endif
