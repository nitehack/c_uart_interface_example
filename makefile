all: mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -I  ../c_library_v1 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread -lmraa

clean:
	 rm -rf *o mavlink_control
