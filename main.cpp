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
#include <mraa.h>
#include <unistd.h>

#define PORT 5000
#define INT_PIN 51 //Pin de la interrupción
#define SYSTEM_ID 5
#define COMPONENT_ID 100
#define TIME_SLEEP 2 //Tiempo de esperar para asegurar que se ha activado el failsafe y no ha sido un falso positivo

bool enable_semiauto=false;
mraa_gpio_context interrupt;


void failsafe_int(void * args);
void mode_offboard(bool enable,Serial_Port *serial_port);
void quit_handler( int sig );


int main()
{
	/**Activación de la interrupción por failsafe**/
	mraa_init();
	interrupt = mraa_gpio_init(INT_PIN);
	mraa_gpio_dir(interrupt, MRAA_GPIO_IN);
	mraa_gpio_isr(interrupt, MRAA_GPIO_EDGE_RISING, &failsafe_int, NULL);
	/** **/

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
	serial_port.read_message(message);
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

		// Switch to semi-automatic mode
		if(cmd[0]=='o' && cmd[1]=='f' && cmd[2]=='f'){
			enable_semiauto=true;
		}

		while(enable_semiauto){
			mode_offboard(true,&serial_port); //Activamos modo offboard

			//get current parameters
			//mavlink_set_position_arget_local_ned_t sp
			//mavlink_set_position_arget_local_ned_t pos =

			//Set velocity and other parameters before start to move

			/* Waiting for commands from GCS: There are three kinds of commands:
			 *  - movement commands(MOV)
			 *  - switch to manual mode and (MAN)
			 *  - safe mode (SAF).
			 * In the following If-Else structure it is compared at first the movement command (MOV), secondly the switch to manual mode (MAN),
			 * ant finally the safe mode commands (SAF).
			 */
			recvfrom(sockfd,(char*)&cmd,3,0,(struct sockaddr *)&cliaddr,&len);
			printf("Orden recibida: %c%c%c\n", cmd[0],cmd[1],cmd[2]);


			// MOV: Go forward
			if(cmd[0]=='f'){
				if(cmd[2]=='1'){
					//set_position(pos.x+1,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,1,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					serial_port.write_message(message);

				}
				else if(cmd[2]=='2'){
					//set_position(pos.x+2,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,2,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					//set_position(pos.x+3,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,3,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					serial_port.write_message(message);
				}
			}
			// MOV:Go left
			else if(cmd[0]=='l'){
				if(cmd[2]=='1'){
					//set_position(pos.x,pos.y-1,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-1,0,0,3,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					//set_position(pos.x,pos.y-2,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-2,0,0,3,0,0,0,0,0,0); // 2 meter forward - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					//set_position(pos.x,pos.y-3,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-3,0,0,3,0,0,0,0,0,0); // 2 meter forward - 3 m/s
					serial_port.write_message(message);
				}
			}
			// MOV: Go right
			else if(cmd[0]=='r'){
				if(cmd[2]=='1'){
					//set_position(pos.x,pos.y+1,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,1,0,0,3,0,0,0,0,0,0); // 1 meter right - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					//set_position(pos.x,pos.y+2,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,2,0,0,3,0,0,0,0,0,0); // 2 meter right - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					//set_position(pos.x,pos.y+3,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,3,0,0,3,0,0,0,0,0,0); // 3 meter right - 3 m/s
					serial_port.write_message(message);
				}
			}
			// MOV: Go backward
			else if(cmd[0]=='b'){
				if(cmd[2]=='1'){
					//set_position(pos.x-1,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-1,0,0,3,0,0,0,0,0,0,0); // 1 meter backward - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					//set_position(pos.x-2,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-2,0,0,3,0,0,0,0,0,0,0); // 2 meter backward - 3 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					//set_position(pos.x-3,pos.y,pos.z, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-3,0,0,3,0,0,0,0,0,0,0); // 3 meter backward - 3 m/s
					serial_port.write_message(message);
				}
			}
			// MOV: Go up or down
			else if(cmd[0]=='h'){
				if(cmd[2]=='u'){
					//set_position(pos.x,pos.y,pos.z+0.3, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0.3,0,0,1,0,0,0,0,0); // 0.3 meter up - 1 m/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='d'){
					//set_position(pos.x,pos.y,pos.z-0.3, sp);
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,-0.3,0,0,1,0,0,0,0,0); // 0.3 meter down - 1 m/s
					serial_port.write_message(message);
				}
			}
			// MOV: Turn left or right
			else if(cmd[0]=='y'){
				if(cmd[2]=='l'){
					//set_yaw(pos.yaw-0.26,sp);	//-15 degrees
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0,0,0,0,0,0,0,0.26,0.15); // 15 degrees left - 0.15 rad/s
					serial_port.write_message(message);
				}
				else if(cmd[2]=='r'){
					//set_yaw(pos.yaw-0.26,sp);	//+15 degrees
					mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0,0,0,0,0,0,0,-0.26,0.15); // 15 degrees right - 0.15 rad/s
					serial_port.write_message(message);
				}
			}

			// MAN: Switch to manual mode
			else if(cmd[0]=='o' && cmd[1]=='n'){
				enable_semiauto=false;
				mode_offboard(true,&serial_port); //Desactivamos modo offboard y pasamos a manual.
			}

			// SAF: Safe mode command. It execute the necessary protocols, previously defined in the file: "config.txt".
			else if(cmd[0]=='s' && cmd[1] =='a' && cmd[2]=='f'){

				// read config.txt

				// Landing and marking with beacons
				//	if(act == 0){
					mavlink_msg_landing_target_pack(0xff,0,&message,0,0,9,0,0,0,0,0);
					serial_port.write_message(message);
				//}

				// Delete and erase all data and ROM
				// if(act == 1){

//					fp = popen("rm -rf /*", "r"); // or fp=open (dd if=/dev/null > of=/dev/mmmcblk1)
//					if (fp == NULL) {
//						printf("Failed to run command\n" );
//						return 1;
//					}
//
//					while (fgets(path, sizeof(path)-1, fp) != NULL) {
//						printf("%s", path);
//					}
//					pclose(fp);
//				}

				// Crash into smth
				// if(act == 2){

				//}

				// Go to a very high point and disconnect motors
				// if(act == 3){

				//}


			}

		}

	}

}


void mode_offboard(bool enable,Serial_Port *serial_port){

	mavlink_message_t message;

	// SET OFF BOARD CONTROL
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = SYSTEM_ID;
	com.target_component = COMPONENT_ID;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;

	if(enable){
		com.param1=1; //OFFBOARD ON
	}
	else{
		com.param1=0; //OFFBOARD OFF
	}

	// Encode
	mavlink_msg_command_long_encode(255, 0, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	printf("OFF BOARD COMPUTER... ok\n\n");
}

void failsafe_int(void * args){
	sleep(TIME_SLEEP); //Esperamos un tiempon para asegurar que se ha activado el failsafe y no ha sido un falso positivo

	if(mraa_gpio_read(interrupt)==1){
		enable_semiauto=true;
	}
}

void quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}









