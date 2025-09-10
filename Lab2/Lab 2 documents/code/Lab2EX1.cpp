//Use g++ -std=c++11 -o Lab2EX1 Lab2EX1.cpp -lwiringPi

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std;
using namespace std::chrono;

int main(){
	//Set up wiringPi
	wiringPiSetup();
	
	while(true){
		/*Set the pinMode to output and generate a LOW-HIGH-LOW signal using "digitalWrite" to trigger the sensor. 
		Use a 2 us delay between a LOW-HIGH and then a 5 us delay between HIGH-LOW. You can use
		the function "usleep" to set the delay. The unit of usleep is microsecond. */
	

		/*Echo holdoff delay 750 us*/



		/*Switch the pinMode to input*/ 
		
		
		
		/*Get the time it takes for signal to leave sensor and come back.*/

		// 1. defind a varable to get the current time t1. Refer to "High_Resolution_Clock_Reference.pdf" for more information

		while(/*read signal pin, the stay is the loop when the signal pin is high*/ )
		{
			// 2. defind a varable to get the current time t2.

			// 3. calculate the time duration: t2 - t1

			// 4. if the duration is larger than the Pulse Maxium 18.5ms, break the loop.
		}



		/*Calculate the distance by using the time duration that you just obtained.*/ //Speed of sound is 340m/s



		/*Print the distance.*/



		/*Delay before next measurement. The actual delay may be a little longer than what is shown is the datasheet.*/


        }
}
