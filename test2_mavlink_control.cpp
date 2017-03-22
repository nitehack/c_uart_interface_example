#include "mavlink_control.h"

//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
int main(int argc, char **argv)
{

    char *uart_name = (char*)"/dev/ttyUSB1";
    int baudrate = 57600;    
    mavlink_message_t message;
    
    int success;


    // Creating a Serial_Port object with the baudrate 57600 on /dev/ttyAMA0
    Serial_Port serial_port(uart_name, baudrate);
    
    // Giving the program 1 second to create the serial_port.
    sleep(1);

    // Starting the serial port
    serial_port.start();

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
    heartbeat.type = MAV_TYPE_GCS;
    mavlink_msg_heartbeat_encode(1, 1, &msg_heartbeat, &heartbeat);


    // SET MESSAGE_INTERVAL. The aims to decrease the time response between MAV message recieved from OBC.
    
    

    // Creating a struct needed to override the rc channels (rco for RC override)
    mavlink_rc_channels_override_t rco;
    
    // declaring a 2 byte variable. 0xFFFF on a channels means that it is unused and therefore not changed
    uint16_t    u = 0xFFFF;

    // Initializing the struct variables of rco
    rco.chan1_raw = 1100; 
    rco.chan2_raw = 1478; 
    rco.chan3_raw = 1700; 
    rco.chan4_raw = 1478; 
    rco.chan5_raw = 1481; 			// Backside mode switch. If you change this value, it will probably change the flight mode.
    rco.chan6_raw = 1481;			// Backside mode switch
    rco.chan7_raw = 1481; 			// Backside mode switch
    rco.chan8_raw = 1481; 			// Backside mode switch 
    rco.target_system =0;
    rco.target_component = 0;

	// Initializing the struct variables of "actuator control"
	/*mavlink_set_actuator_control_target_t control;
	
	control.time_usec = 1000;
	control.group_mlx = 0;
	control.controls[0] = 0;
	control.controls[1] = 0;
	control.controls[2] = 0;
	control.controls[3] = -1;
	control.controls[4] = 0;
	control.controls[5] = 0;
	control.controls[6] = 0;
	control.controls[7] = 0;
	control.target_system = 0;
	control.target_component = 0;
	*/
    
    // Encoding the struct into bytes
    //printf ("\nStart sending ...\n");
    mavlink_msg_rc_channels_override_encode(1,1,&message, &rco);
    for(int i = 0; i<10; i++){
    	   // CHANEL OVERRIDE
        serial_port.write_message(msg_heartbeat);
        
        serial_port.write_message(message);
        
        // MANUAL CONTROL  
        //mavlink_msg_manual_control_pack(0xff,0, &message, MAV_TYPE_COAXIAL, -900, -900, -900, -900, 0);   
        //int len = serial_port.write_message(message);
        
        // SET ACTUATOR CONTROL
        //mavlink_msg_set_actuator_control_target_encode(0xff, 0, &message, controls)
        //int len = serial_port.write_message(message);
    
        sleep(1);
    }
    printf("Finish \n\n");

    // Directly write the PWM values of the individual channels into the function

        //mavlink_msg_rc_channels_override_pack(0xff,0, &message,0,0,u,u,u,1000,u,u,u,u );
        //int len = serial_port.write_message(message);


    // All channels are set to 0. This releases the channels and give the control back to the RC
        //mavlink_msg_rc_channels_override_pack(0xff,0, &message,0,0,0,0,0,0,0,0,0,0 );
        //len = serial_port.write_message(message);
    

    // Stoping the serial_port (closing it)
    serial_port.stop();
    sleep(1);
    return 0;
}
