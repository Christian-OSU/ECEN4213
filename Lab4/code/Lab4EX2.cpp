//use g++ -std=c++11 -o Lab4EX2 Lab4EX2.cpp -lwiringPi


#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std;
using namespace std::chrono;

int kobuki;
float read_sonar();
void movement(int, int);
int pin = 1; // sonar pin
int flag = 0; // flag to indicate if obstacle is detected
int sp = 200; // speed
float dist = 100;

int main(){
	wiringPiSetup();
	kobuki = serialOpen("/dev/kobuki", 115200);

	/*Move from a random point within the area designated "X" to the
	point B as shown on the diagram. Use a sonar sensor to navigate through the channel.
	You can reuse your code from Lab 2 and 3*/

	/*Note: the Kobuki must completely pass point B as shown to receive full credit*/
	while (true){
		dist = read_sonar();
		if (dist < 15 && flag == 0){
			movement(100,-1);
			usleep(2550000);
			flag = 1;
		}
		else if (dist < 15 && flag == 1){
			movement(100,1);
			usleep(2550000);
			flag = 2;
		}
		else {
			movement(sp,0);
		}
		cout << "DIST: " << dist << endl;

	}
		


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