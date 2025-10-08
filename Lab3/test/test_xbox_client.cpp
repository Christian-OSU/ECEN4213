#include <stdio.h>
#include <iostream>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>

#define PORT 8080
using namespace std;

int createSocket();
void printJoystickEvent(struct js_event e);

int main() {
    int joy_fd;
    struct js_event e;
    int horiz = 0;
    int vert = 0;
    
    // Open joystick
    joy_fd = open("/dev/input/js0", O_RDONLY);
    if (joy_fd == -1) {
        cout << "Could not open joystick" << endl;
        return -1;
    }
    
    // Create socket connection
    int sock = createSocket();
    if (sock < 0) {
        cout << "Socket connection failed" << endl;
        close(joy_fd);
        return -1;
    }
    
    cout << "Xbox controller connected. Controls:" << endl;
    cout << "Left stick up/down: Forward/Backward" << endl;
    cout << "Left stick left/right: Turn left/right" << endl;
    cout << "Back button: Exit" << endl;
    
    while (true) {
        // Read joystick event
        if (read(joy_fd, &e, sizeof(e)) > 0) {
            printJoystickEvent(e);
            
            // Process events
            if (e.type == JS_EVENT_AXIS) {
                switch(e.number) {
                    case 0: // Left stick horizontal
                        horiz = (e.value > 16384) ? 1 : (e.value < -16384) ? -1 : 0;
                        break;
                    case 1: // Left stick vertical
                        vert = (e.value > 16384) ? -1 : (e.value < -16384) ? 1 : 0;
                        break;
                }
            }
            else if (e.type == JS_EVENT_BUTTON && e.value == 1) {
                if (e.number == 6) { // Back button
                    cout << "Exiting..." << endl;
                    close(sock);
                    close(joy_fd);
                    return 0;
                }
            }
            
            // Send control data
            string message = to_string(horiz) + "," + to_string(vert);
            cout << "Sending: " << message << endl;
            send(sock, message.c_str(), message.length(), 0);
        }
    }
    
    return 0;
}

void printJoystickEvent(struct js_event e) {
    cout << "Event: ";
    if (e.type == JS_EVENT_BUTTON) {
        cout << "Button " << (int)e.number << " = " << e.value << endl;
    }
    else if (e.type == JS_EVENT_AXIS) {
        cout << "Axis " << (int)e.number << " = " << e.value << endl;
    }
}

int createSocket() {
    int sock = 0;
    struct sockaddr_in serv_addr;
    
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\nSocket creation error \n");
        return -1;
    }
    
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
    
    return sock;
}
