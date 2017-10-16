all: mavlink_control test2_mavlink_control main

main: main.cpp git_submodule
	g++ -I mavlink/include/mavlink/v1.0 main.cpp serial_port.cpp -o main -lpthread

test2_mavlink_control: test2_mavlink_control.cpp git_submodule
	g++ -I mavlink/include/mavlink/v1.0 test2_mavlink_control.cpp serial_port.cpp -o test2_mavlink_control -lpthread

mavlink_control: git_submodule mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread
	
git_submodule:
	git submodule update --init  --remote --merge

clean:
	 rm -rf *o mavlink_control
