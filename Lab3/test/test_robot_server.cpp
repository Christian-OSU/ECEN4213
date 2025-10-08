#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define PORT 8080
using namespace std;

void movement(int speed, int radius) {
    cout << "ROBOT COMMAND - Speed: " << speed << "mm/s, Radius: " << radius << "mm" << endl;
    if (speed == 0 && radius == 0) {
        cout << "Status: Stopped" << endl;
    } else if (radius == 0) {
        cout << "Status: " << (speed > 0 ? "Moving Forward" : "Moving Backward") << endl;
    } else {
        cout << "Status: Turning " << (radius > 0 ? "Right" : "Left") << endl;
    }
    cout << "------------------------" << endl;
}

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    
    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    
    cout << "Test Robot Server Started" << endl;
    cout << "Waiting for controller connection..." << endl;
    
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    
    cout << "Controller connected!" << endl;
    
    while(true) {
        int valread = read(new_socket, buffer, 1024);
        if (valread == 0) {
            cout << "Controller disconnected" << endl;
            break;
        }
        
        cout << "Received: " << buffer << endl;
        
        char *token = strtok(buffer, ",");
        if (token != NULL) {
            int horiz = atoi(token);
            token = strtok(NULL, ",");
            if (token != NULL) {
                int vert = atoi(token);
                
                int speed = 0;
                int radius = 0;
                
                if (vert != 0) {
                    speed = vert * 200;
                }
                
                if (horiz != 0) {
                    if (horiz == 1) {
                        radius = 500;
                    } else {
                        radius = -500;
                    }
                    if (speed == 0) {
                        speed = 150;
                    }
                }
                
                movement(speed, radius);
            }
        }
        
        memset(buffer, 0, sizeof(buffer));
    }
    
    close(new_socket);
    return 0;
}
