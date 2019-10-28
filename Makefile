OBJ=node.o CTime.o CSerialConnection.o CLidarPacketSender.o CArcTime.o CLidarPacketReceiver.o CLidarPacket.o CCountDown.o C3iroboticsLidar.o  CDeviceConnection.o modify.o uart.o carcontrol.o
#CC=g++
CC=arm-linux-gnueabihf-g++
#lidar_ubuntu:$(OBJ)
lidar_de1:$(OBJ)
#	$(CC) -lrt -pthread -o lidar_ubuntu $(OBJ)
	$(CC) -lrt -pthread -o lidar_de1 $(OBJ)

node.o:node.cpp
	$(CC) -lrt -pthread -c node.cpp

modify.o:modify.cpp
	$(CC) -lrt -pthread -c modify.cpp

uart.o:uart.cpp
	$(CC) -lrt -pthread -c uart.cpp

carcontrol.o:carcontrol.cpp
	$(CC) -lrt -pthread -c carcontrol.cpp
    
CTime.o:CTime.cpp
	$(CC) -lrt -pthread -c CTime.cpp

CSerialConnection.o:CSerialConnection.cpp
	$(CC) -lrt -pthread -c CSerialConnection.cpp

CLidarPacketSender.o:CLidarPacketSender.cpp
	$(CC) -lrt -pthread -c CLidarPacketSender.cpp

CLidarPacketReceiver.o:CLidarPacketReceiver.cpp
	$(CC) -lrt -pthread -c CLidarPacketReceiver.cpp

CLidarPacket.o:CLidarPacket.cpp
	$(CC) -lrt -pthread -c CLidarPacket.cpp

CArcTime.o:CArcTime.cpp
	$(CC) -lrt -pthread -c CArcTime.cpp

C3iroboticsLidar.o:C3iroboticsLidar.cpp
	$(CC) -lrt -pthread -c C3iroboticsLidar.cpp

CDeviceConnection.o:CDeviceConnection.cpp
	$(CC) -lrt -pthread -c CDeviceConnection.cpp

CCountDown.o:CCountDown.cpp
	$(CC) -lrt -pthread -c CCountDown.cpp



.PHONY: clean 
clean:
	rm -f $(TARGET) *.a *.o *~  
