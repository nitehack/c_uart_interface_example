/***************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include <mraa.h>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
bool enable_semiauto=false;
mraa_gpio_context interrupt;

#define PORT 5000
#define INT_PIN 51 //Pin de la interrupción
#define SYSTEM_ID 5
#define COMPONENT_ID 100
#define TIME_SLEEP 2 //Tiempo de esperar para asegurar que se ha activado el failsafe y no ha sido un falso positivo


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	char *uart_name = (char*)"/dev/ttyS1";
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	autopilot_interface.start(); //<--- Abre las hebras


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface); //<--Comandos



	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
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


	printf("Waiting for commands:\n");

	while(1){

		n = recvfrom(sockfd,(char*)&cmd,3,0,(struct sockaddr *)&cliaddr,&len);
		printf("(ON) Orden recibida: %c%c%c\n", cmd[0],cmd[1],cmd[2]);

		// Switch to semi-automatic mode
		if(cmd[0]=='o' && cmd[1]=='f' && cmd[2]=='f'){
			enable_semiauto=true;
		}

		while(enable_semiauto){
			

			// --------------------------------------------------------------------------
			//   START OFFBOARD MODE
			// --------------------------------------------------------------------------

			api.enable_offboard_control();
			usleep(100); // give some time to let it sink in

				// initialize command data strtuctures
			mavlink_set_position_target_local_ned_t sp;
			mavlink_set_position_target_local_ned_t ip = api.initial_position;



			/* Waiting for commands from GCS: There are three kinds of commands:
			 *  - movement commands(MOV)
			 *  - switch to manual mode and (MAN)
			 *  - safe mode (SAF).
			 * In the following If-Else structure it is compared at first the movement command (MOV), secondly the switch to manual mode (MAN),
			 * ant finally the safe mode commands (SAF).
			 */
			recvfrom(sockfd,(char*)&cmd,3,0,(struct sockaddr *)&cliaddr,&len);
			printf("(OFF) Orden recibida: %c%c%c\n", cmd[0],cmd[1],cmd[2]);


			// MOV: Go forward
			if(cmd[0]=='f'){
				if(cmd[2]=='1'){
					set_position(1,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,1,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					//serial_port.write_message(message);

				}
				else if(cmd[2]=='2'){
					set_position(2,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,2,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					set_position(3,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,3,0,0,3,0,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					//serial_port.write_message(message);
				}
			}
			// MOV:Go left
			else if(cmd[0]=='l'){
				if(cmd[2]=='1'){
					set_position(0,-1,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-1,0,0,3,0,0,0,0,0,0); // 1 meter forward - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					set_position(0,-2,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-2,0,0,3,0,0,0,0,0,0); // 2 meter forward - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					set_position(0,-3,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,-3,0,0,3,0,0,0,0,0,0); // 2 meter forward - 3 m/s
					//serial_port.write_message(message);
				}
			}
			// MOV: Go right
			else if(cmd[0]=='r'){
				if(cmd[2]=='1'){
					set_position(0,1,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,1,0,0,3,0,0,0,0,0,0); // 1 meter right - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					set_position(0,2,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,2,0,0,3,0,0,0,0,0,0); // 2 meter right - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					set_position(0,3,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,3,0,0,3,0,0,0,0,0,0); // 3 meter right - 3 m/s
					//serial_port.write_message(message);
				}
			}
			// MOV: Go backward
			else if(cmd[0]=='b'){
				if(cmd[2]=='1'){
					set_position(-1,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-1,0,0,3,0,0,0,0,0,0,0); // 1 meter backward - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='2'){
					set_position(-2,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-2,0,0,3,0,0,0,0,0,0,0); // 2 meter backward - 3 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='3'){
					set_position(-3,0,0, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,-3,0,0,3,0,0,0,0,0,0,0); // 3 meter backward - 3 m/s
					//serial_port.write_message(message);
				}
			}
			// MOV: Go up or down
			else if(cmd[0]=='h'){
				if(cmd[2]=='u'){
					set_position(0,0,0.3, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0.3,0,0,1,0,0,0,0,0); // 0.3 meter up - 1 m/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='d'){
					set_position(0,0,-0.3, sp);
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,-0.3,0,0,1,0,0,0,0,0); // 0.3 meter down - 1 m/s
					//serial_port.write_message(message);
				}
			}
			// MOV: Turn left or right
			else if(cmd[0]=='y'){
				if(cmd[2]=='l'){
					set_yaw(-0.26,sp);	//-15 degrees
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0,0,0,0,0,0,0,0.26,0.15); // 15 degrees left - 0.15 rad/s
					//serial_port.write_message(message);
				}
				else if(cmd[2]=='r'){
					set_yaw(-0.26,sp);	//+15 degrees
					//mavlink_msg_set_position_target_local_ned_pack(0xff,0,&message,0,5,100,9,0,0,0,0,0,0,0,0,0,0,-0.26,0.15); // 15 degrees right - 0.15 rad/s
					//serial_port.write_message(message);
				}
			}
			api.update_setpoint(sp); //actualizamos posición

			// MAN: Switch to manual mode
			if(cmd[0]=='o' && cmd[1]=='n'){
				enable_semiauto=false;
				api.disable_offboard_control();
				//mode_offboard(true,&serial_port); //Desactivamos modo offboard y pasamos a manual.
			}

			// SAF: Safe mode command. It execute the necessary protocols, previously defined in the file: "config.txt".
			else if(cmd[0]=='s' && cmd[1] =='a' && cmd[2]=='f'){

				// read config.txt

				// Landing and marking with beacons
				//	if(act == 0){

				/* MIRAR ESTO
					mavlink_msg_landing_target_pack(0xff,0,&message,0,0,9,0,0,0,0,0);
					serial_port.write_message(message);
				*/


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

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

// ------------------------------------------------------------------------------
//   FAIL SAFE trigger
// ------------------------------------------------------------------------------
void failsafe_int(void * args){
	sleep(TIME_SLEEP); //Esperamos un tiempon para asegurar que se ha activado el failsafe y no ha sido un falso positivo

	if(mraa_gpio_read(interrupt)==1){
		enable_semiauto=true;
	}
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
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


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


