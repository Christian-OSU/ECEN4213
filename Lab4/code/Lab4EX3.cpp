//Use g++ -std=c++11 -o Lab4EX3 Lab4EX3.cpp -lwiringPi

#include <string>
#include <iostream>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std;
using namespace std::chrono;

int kobuki;

unsigned int bumper;
unsigned int drop;
unsigned int cliff;
unsigned int button;
unsigned int readB; 
unsigned int flag = 0;
int dist;
int pin =1;

void movement(int, int);
void readData();
float read_sonar();

int main(){
	//Create connection to the Kobuki
	wiringPiSetup();
	kobuki = serialOpen("/dev/kobuki", 115200);

	while(serialDataAvail(kobuki) != -1){
		/*Read the initial data. If there are no flags,
		the default condition is forward.*/
		/*Move slowly to give the sensors enough time to read data,
		the recommended speed is 100mm/s*/
		readData();
		dist = read_sonar();

		if (cliff >= 1 && cliff <= 11) {
			movement(-150, 0); 
			for (int i = 0; i < 50; i++) {
				usleep(20000); 
				readData();    
			}
			movement(100, -1);
			for (int i = 0; i < 50; i++) {
				usleep(20000);
				readData();    
			}
		}
		else if (bumper >= 1 && bumper <= 10){
			movement(-150, 0); 
			for (int i = 0; i < 25; i++) {
				usleep(20000); 
				readData();    
			}
			movement(100, -1);
			for (int i = 0; i < 50; i++) {
				usleep(20000);
				readData();    
			}
		}
		/*else if (dist < 15){
			movement(-150, 0); 
			for (int i = 0; i < 25; i++) {
				usleep(20000); 
				readData();    
			}
			movement(100, -1);
			for (int i = 0; i < 50; i++) {
				usleep(20000);
				readData();    
			}
		}*/
		else {
			movement(90,0);
		}


		/*Create different states as to satisfy the conditions above.
		Remember, a single press of a bumper may last longer
		than one data cycle.*/

		/*Cleanly close out of all connections using Button 1.*/

		/*Use serialFlush(kobuki) to discard all data received, or waiting to be send down the given device.*/
	}
}

void movement(int sp, int r){
	// you can reuse your code from Lab 3
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

void readData(){
	// you can reuse your code from EXE1, Lab 4
	//If the bytes are a 1 followed by 15, then we are
		//parsing the basic sensor data packet
		while(true){
			readB = serialGetchar(kobuki);
		if(readB == 1){
			if(serialGetchar(kobuki) == 15) {break;}
		}
	}

	//Read past the timestamp
	serialGetchar(kobuki);
	serialGetchar(kobuki);

	/*Read the bytes containing the bumper, wheel drop,
		and cliff sensors. You can convert them into a usable data type.*/
	bumper = serialGetchar(kobuki);
	drop = serialGetchar(kobuki);
	cliff = serialGetchar(kobuki);

	/*Print the data to the screen.*/
	cout << "Bumper: " << bumper << ", " << "Drop: " << drop << ", " << "Cliff: " << cliff << endl;

	/*Read through 6 bytes between the cliff sensors and
	the button sensors.*/
	serialGetchar(kobuki);
	serialGetchar(kobuki);
	serialGetchar(kobuki);
	serialGetchar(kobuki);
	serialGetchar(kobuki);
	serialGetchar(kobuki);

	/*Read the byte containing the button data.*/
	button = serialGetchar(kobuki);
	/*Close the script and the connection to the Kobuki when
	Button 1 on the Kobuki is pressed. Use serialClose(kobuki);*/
	if (button == 1) {
		serialClose(kobuki);
		exit(0);
	}

	

	//Pause the script so the data read receive rate is the same as the Kobuki send rate.
	usleep(20000);
}

float read_sonar(){
		// you can reuse your code from Lab 2
		/*Set the pinMode to output and generate a LOW-HIGH-LOW signal using "digitalWrite" to trigger the sensor. 
			Use a 2 us delay between a LOW-HIGH and then a 5 us delay between HIGH-LOW. You can use
			the function "usleep" to set the delay. The unit of usleep is microsecond. */
			high_resolution_clock::time_point t1;
			float pulse_width;
			high_resolution_clock::time_point t2;
			pinMode(pin, OUTPUT);
			digitalWrite(pin, LOW);
			//cout << digitalRead(pin);
			usleep(2);
			digitalWrite(pin,HIGH);
			//cout << digitalRead(pin);
			usleep(5);
			digitalWrite(pin,LOW);
			//cout << digitalRead(pin);
		
	
			/*Echo holdoff delay 750 us*/
			usleep(750);  
	
	
			/*Switch the pinMode to input*/ 
			pinMode(pin, INPUT);
			//cout <<"Value: " << digitalRead(pin) << endl;
			
			
			
			/*Get the time it takes for signal to leave sensor and come back.*/
	
			// 1. defind a varable to get the current time t1. Refer to "High_Resolution_Clock_Reference.pdf" for more information
			//while(digitalRead(pin) == LOW){
				t1 = high_resolution_clock::now();
			//}
			
			while(digitalRead(pin) == HIGH)
			{
				// 2. defind a varable to get the current time t2.
				t2 = high_resolution_clock::now();
				// 3. calculate the time duration: t2 - t1
				pulse_width = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
				//cout << "Pulse_Width: " << pulse_width << endl;
				// 4. if the duration is larger than the Pulse Maxium 18.5ms, break the loop.
				if (pulse_width > 18500) {
					break;
				
			}
	
	
	
			/*Calculate the distance by using the time duration that you just obtained.*/ //Speed of sound is 340m/s
				dist = pulse_width / 58;
	
	
			/*Print the distance.*/
				//cout << "measured value: " << dist << endl;
	
	
			/*Delay before next measurement. The actual delay may be a little longer than what is shown is the datasheet.*/
	
			}
			return dist;
	
	}
