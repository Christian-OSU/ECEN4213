
//Use g++ -std=c++11 -o Lab3EX3A Lab3EX3A.cpp -lwiringPi

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#define PORT 8080
using namespace std;

void movement(int, int);
void createSocket();
void readData();

int kobuki, new_socket;

/*Create char buffer to store transmitted data*/

int main(){
	//Initialize filestream for the Kobuki
	wiringPiSetup();
	//kobuki = serialOpen("/dev/kobuki", 115200);
	kobuki = serialOpen("/dev/ttyUSB0", 115200);


	//Create connection to client
	createSocket();

	while(true){
		//Read data from client
		readData();
	}
	return 0;
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

	/*Send the data (Byte 1 - Byte 9) to Kobuki using serialPutchar (kobuki, );*/
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

//Creates the connection between the client and
//the server with the Kobuki being the server
void createSocket(){
	int server_fd;
	struct sockaddr_in address;
	int opt =1;
	int addrlen = sizeof(address);

	if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))){
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family      = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port        = htons(PORT);

	if(bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0){
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	if(listen(server_fd, 3) < 0){
		perror("listen");
		exit(EXIT_FAILURE);
	}

	if((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0){
		perror("accept");
		exit(EXIT_FAILURE);
	}
}

void readData() {
    /*Read the incoming data stream from the controller*/
    char buffer[1024] = {0};
    int valread = read(new_socket, buffer, sizeof(buffer));

    
    /*Print the received data*/
    printf("Received: %s\n", buffer);

    /*Parse the Horiz,Vert values (e.g., "1,-1")*/
    int horiz = 0, vert = 0, stop = 0, shouldClose = 0;
    sscanf(buffer, "%d,%d,%d,%d", &horiz, &vert, &stop, &shouldClose);

	/*Check if client disconnected*/
    if (valread <= 0 || shouldClose == 1) {
        printf("Client disconnected\n");
        close(new_socket);
        serialClose(kobuki);
        exit(0);
    }


    /*Convert to Kobuki movement parameters*/
    int speed = 0;
    int radius = 0;

    

	if (stop == 1) {
		speed = 0;     // stop
		radius = 0;    // go straight
	}
	else {
		// Speed control (forward/backward)
    speed = (float(vert) / 32767) * 300 * -1;
	radius = (float(horiz) / 32767) * 300 * -1;
	}
	cout << "Speed: " << speed << " Radius: " << radius << endl;

    /*Use the received data to control the Kobuki*/
    movement(speed, radius);

    /*Reset the buffer*/
    memset(buffer, 0, sizeof(buffer));
}
