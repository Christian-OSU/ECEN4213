//Christian Landrum Wyatt Probst Group 5
//use g++ -std=c++11 -o Lab3EX1 Lab3EX1.cpp -lwiringPi

#include <iostream>
#include <unistd.h>
#include <wiringSerial.h>
#include <wiringPi.h>
using namespace std;

void movement(int, int);
void stopKobuki();
int kobuki;

int main(){
	wiringPiSetup();

	kobuki = serialOpen("/dev/kobuki", 115200);
	if(kobuki != -1) std::cout << "Serial port opened" << std::endl;
	
	//The Kobuki accepts data packets at a rate of 20 ms.
	//To continually move, data needs to be sent continuously. Therefore, 
	//you need to call void movement(int sp, int r) in a for or while loop 
	//in order to run a specific distance.
	int sp = 100; //Speed in mm/s
	int r = 500; //Radius in mm
	int b = 230; //Length between the center of the wheels in mm
	int w = 2; //Rotation speed in rad/s

	//Due to machine error, the calculated value of the time needed
	//will not be exact, but can give you a rough starting value.
	//Note: stop the robot movement between each movement segment.
	movement(0,0);


	/*Rotate the Kobuki 90 degrees*/
	for (int i=0; i <1; i++){
		movement(w*b/2,1);
		usleep(1150000);
	}
	
	

	/*Move along the vertical side*/
	
	for(int i=0; i<5; i++){
		movement(sp,0);
		sleep(1);
	}

	/*Move along quarter circle*/
	for (int i=0; i<8; i++){
		movement((sp*((r+b/2)/r)),-500);
		sleep(1);
	}

	//Close the serial stream to the Kobuki
	//Not doing this will result in an unclean disconnect
	//resulting in the need to restart the Kobuki
	//after every run.
	serialClose(kobuki);
	return(0);
}

void movement(int sp, int r){
	//Create the byte stream packet with the following format:
	unsigned char b_0 = 0xAA; /*Byte 0: Kobuki Header 0*/
	unsigned char b_1 = 0x55; /*Byte 1: Kobuki Header 1*/
	unsigned char b_2 = 0x06; /*Byte 2: Length of Payload*/
	unsigned char b_3 = 0x01; /*Byte 3: Sub-Payload Header (Base control)*/
	unsigned char b_4 = 0x04; /*Byte 4: Length of Sub-Payload*/ 

	unsigned char b_5 = sp & 0xff;	//Byte 5: Payload Data: Speed(mm/s)
	unsigned char b_6 = (sp >> 8) & 0xff; //Byte 6: Payload Data: Speed(mm/s)
	unsigned char b_7 = r & 0xff;	//Byte 7: Payload Data: Radius(mm)
	unsigned char b_8 = (r >> 8) & 0xff;	//Byte 8: Payload Data: Radius(mm)
	unsigned char checksum = 0;		//Byte 9: Checksum
	
	//Checksum all of the data
	char packet[] = {b_0,b_1,b_2,b_3,b_4,b_5,b_6,b_7,b_8};
	for (unsigned int i = 2; i < 9; i++)
		checksum ^= packet[i];

	/*Send the data (Byte 0 - Byte 8 and checksum) to Kobuki using serialPutchar (kobuki, );*/
	serialPutchar(kobuki, b_0);
	serialPutchar(kobuki, b_1);
	serialPutchar(kobuki, b_2);
	serialPutchar(kobuki, b_3);
	serialPutchar(kobuki, b_4);
	serialPutchar(kobuki, b_5);
	serialPutchar(kobuki, b_6);
	serialPutchar(kobuki, b_7);
	serialPutchar(kobuki, b_8);
	serialPutchar(kobuki, checksum);


	/*Pause the script so the data send rate is the
	same as the Kobuki data receive rate*/
	usleep(20000);

}

// Function to stop the Kobuki
void stopKobuki() {
    movement(0, 0); // Send zero speed to stop
    usleep(100000); // Wait for 100 ms to ensure the stop command is received
}

