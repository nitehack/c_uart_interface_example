/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "mavlink_control.h"
#include "time.h"

#define PORT 5000

int main()
{
	int sockfd, n;
	struct sockaddr_in servaddr,cliaddr;
	socklen_t len;
	int iSetOption = 1;

	char cmd[3];

	char *uart_name = (char*)"/dev/ttyUSB1";
	int baudrate = 921600;
	mavlink_message_t message;


	/** Init Sockets UDP **/
	sockfd=socket(AF_INET,SOCK_DGRAM,0);
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,(char*)&iSetOption, sizeof(iSetOption));
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
	servaddr.sin_port=htons(PORT);
	if(bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0){

		if( errno == EADDRINUSE ){
			printf("the port is not available. already to other process\n");
		} else {
			printf("could not bind to process (%d) %s\n", errno, strerror(errno));
		}
	}
	len = sizeof(cliaddr);


	/** Init UART - MAVLINK Interface **/
	// Creating a Serial_Port object with the baudrate 57600 on /dev/ttyAMA0
	Serial_Port serial_port(uart_name, baudrate);

	// Giving the program 1 second to create the serial_port.
	sleep(1);

	//Create autopilot interface
	Autopilot_Interface autopilot_interface(&serial_port);

	// Setup interrupt signal handler
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	// Starting the serial port and autopilot_interface
	serial_port.start();
	autopilot_interface.start();

	// Giving the program 1 second to start the serial port
	sleep(1);


	// HEARTBEAT messages from Pixhawk
	success = serial_port.read_message(message);
	if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
			printf("ID: %i\n", message.sysid);
		printf("COMPONENT: %i\n", message.compid);
	}

	mavlink_heartbeat_t heartbeat;
	mavlink_message_t msg_heartbeat;
	//heartbeat.type = MAV_TYPE_COAXIAL;

	mavlink_msg_heartbeat_pack(255, 0, &msg_heartbeat, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 5, 100, 0);


	//printf("Waiting for commands:\n");

	while(1){

		n = recvfrom(sockfd,(char*)&cmd,3,0,(struct sockaddr *)&cliaddr,&len);
		printf("Orden recibida: %c%c%c\n", cmd[0],cmd[1],cmd[2]);

		//Switch to semi-automatic state
		if(cmd[0]=='o' && cmd[1]=='f' && cmd[2]=='f'){
			mode_offboard(&serial_port);

			//get current parameters
			//mavlink_set_position_arget_local_ned_t sp
			//mavlink_set_position_arget_local_ned_t pos =

			//Set velocity and other parameters before start to move

			//Waiting for navigation commands
			recvfrom(sockfd,(char*)&cmd,3,0,(struct sockaddr *)&cliaddr,&len);
			printf("Órden recibida: %c%c%c\n", cmd[0],cmd[1],cmd[2]);

			//Go forward
			if(cmd[0]=='f'){
				if(cmd[2]=='1'){
					//set_position(pos.x+1,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,1,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					serial_port.write_message(message);

				}
				else if(cmd[2]=='2'){
					set_position(pos.x+2,pos.y,pos.z, sp);
				}
				else if(cmd[2]=='3'){
					set_position(pos.x+3,pos.y,pos.z, sp);
				}
			}
			//Go left
			else if(cmd[0]=='l'){
				if(cmd[2]=='1'){
					set_position(pos.x,pos.y-1,pos.z, sp);
				}
				else if(cmd[2]=='2'){
					set_position(pos.x,pos.y-2,pos.z, sp);
				}
				else if(cmd[2]=='3'){
					set_position(pos.x,pos.y-3,pos.z, sp);
				}
			}
			//Go right
			else if(cmd[0]=='r'){
				if(cmd[2]=='1'){
					set_position(pos.x,pos.y+1,pos.z, sp);
				}
				else if(cmd[2]=='2'){
					set_position(pos.x,pos.y+2,pos.z, sp);
				}
				else if(cmd[2]=='3'){
					set_position(pos.x,pos.y+3,pos.z, sp);
				}
			}
			//Go backward
			else if(cmd[0]=='b'){
				if(cmd[2]=='1'){
					set_position(pos.x-1,pos.y,pos.z, sp);
				}
				else if(cmd[2]=='2'){
					set_position(pos.x-2,pos.y,pos.z, sp);
				}
				else if(cmd[2]=='3'){
					set_position(pos.x-3,pos.y,pos.z, sp);
				}
			}
			// Go up or down
			else if(cmd[0]=='h'){
				if(cmd[2]=='u'){
					set_position(pos.x,pos.y,pos.z+0.3, sp);
				}
				else if(cmd[2]=='d'){
					set_position(pos.x,pos.y,pos.z-0.3, sp);
				}
			}
			// Turn left or right
			else if(cmd[0]=='y'){
				if(cmd[2]=='l'){
					set_yaw(pos.yaw-0.26,sp);	//-15 degrees
				}
				else if(cmd[2]=='r'){
					set_yaw(pos.yaw-0.26,sp);	//+15 degrees
				}
			}

		}

	}

}


void mode_offboard(Serial_Port *serial_port){

	mavlink_message_t message;

	// SET OFF BOARD CONTROL
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = 5;
	com.target_component = 100;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = 1;

	// Encode
	mavlink_msg_command_long_encode(255, 0, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	printf("OFF BOARD COMPUTER... ok\n\n");
}













