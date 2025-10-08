void readData(){
    /*Read the incoming data stream from the controller*/
    char buffer[1024] = {0};
    int valread = read(new_socket, buffer, 1024);
    
    /*Print the data to the terminal*/
    printf("Received: %s\n", buffer);

    /*Parse the Horiz,Vert values*/
    char *token = strtok(buffer, ",");
    if (token != NULL) {
        int horiz = atoi(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
            int vert = atoi(token);
            
            /*Convert to Kobuki movement parameters*/
            int speed = 0;
            int radius = 0;
            
            // Speed control (forward/backward)
            if (vert != 0) {
                speed = vert * 200; // 200mm/s base speed
            }
            
            // Turning control
            if (horiz != 0) {
                if (horiz == 1) {
                    radius = 500;  // Turn right
                } else {
                    radius = -500; // Turn left
                }
                
                // If we're not moving forward/backward but turning, add some speed
                if (speed == 0) {
                    speed = 150;
                }
            } else {
                radius = 0; // Go straight
            }
            
            /*Use the received data to control the Kobuki*/
            movement(speed, radius);
        }
    }

    /*Check for shutdown command (will be an empty message when client disconnects)*/
    if (valread == 0) {
        printf("Client disconnected\n");
        close(new_socket);
        serialClose(kobuki);
        exit(0);
    }

    /*Reset the buffer*/
    memset(&buffer, '0', sizeof(buffer));
}
