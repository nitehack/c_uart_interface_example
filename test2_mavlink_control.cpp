#include "mavlink_control.h"
#include "time.h"

//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
int main(int argc, char **argv)
{

    char *uart_name = (char*)"/dev/ttyUSB1";
    int baudrate = 921600;    
    mavlink_message_t message;
    
    int success;
    
    time_t timestamp;
    time_t timeheart;


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
    //heartbeat.type = MAV_TYPE_COAXIAL;
    
    mavlink_msg_heartbeat_pack(255, 0, &msg_heartbeat, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 5, 100, 0);


    // SET MESSAGE_INTERVAL. The aims to decrease the time response between MAV message recieved from OBC.
    // todo --> another solution
        

    // Creating a struct needed to override the rc channels (rco for RC override)
    mavlink_rc_channels_override_t rco;
    
    // Control actuator vector
    float controls[8];
    
    // declaring a 2 byte variable. 0xFFFF on a channels means that it is unused and therefore not changed
    uint16_t    u = 0xFFFF;

    // Initializing the struct variables of rco
    rco.chan1_raw = 0; 
    rco.chan2_raw = 0; 
    rco.chan3_raw = 1700; 
    rco.chan4_raw = 0; 
    rco.chan5_raw = 1481; 			// Backside mode switch. If you change this value, it will probably change the flight mode.
    rco.chan6_raw = 1481;			// Backside mode switch
    rco.chan7_raw = 1481; 			// Backside mode switch
    rco.chan8_raw = 1481; 			// Backside mode switch 
    rco.target_system =5;
    rco.target_component = 100;
    
    
    // Encoding the struct into bytes
    //mavlink_msg_rc_channels_override_encode(255,0,&message, &rco);
	
	// Set control actuator vector values
	controls[0] = -1.0;
	controls[1] = -1.0;
	controls[2] = 0.0;
	controls[3] = 0.0;
	controls[4] = 0.0;
	controls[5] = 0.0;
	controls[6] = 0.0;
	controls[7] = 0.0;
	
    timeheart = time(0);
    
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
	int len = serial_port.write_message(message);
	
	printf("OFF BOARD COMPUTER... ok\n\n");
	sleep(1);
       
    printf ("\nStart sending ...\n");
    for(int i = 0; i<1; i++){
    	   
    	   // CHANEL OVERRIDE
    	   //if(difftime(time(0),timeheart) > 1){			//Every second, the heartbeat message is sent.
    	   // 	 serial_port.write_message(msg_heartbeat); 
    	   //	 timeheart = time(0);
    	   //}    	   
               	   									
        //serial_port.write_message(message);
        
        // MANUAL CONTROL  
        //mavlink_msg_manual_control_pack(0xff,0, &message, MAV_TYPE_COAXIAL, -900, -900, -900, -900, 0);   
        //int len = serial_port.write_message(message);
        
        // SET ACTUATOR CONTROL
        mavlink_msg_set_actuator_control_target_pack(0xff, 0, &message, 150000, 0, 5, 100, controls);
        int len = serial_port.write_message(message);
        sleep(2);
        
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
