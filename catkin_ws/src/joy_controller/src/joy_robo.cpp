/* 	==============================================================
	==============================================================
							NANTROBOT
				Author: Alexi MARTIN and Dario PALMA
					Last modification: 22/11/2015
	==============================================================
	==============================================================
							INCLUDES
	==============================================================
*/
/* ROS libraries */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Point32.h"
#include "joy_controller/joystick.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <sstream>

/* C libraries */
#include <stdlib.h>
#include <stdio.h>
#include "time.h"

#define UNDEFINED 0
#define CLOSED 1
#define OPEN 2

#define FrontPlierUndefined 0
#define FrontPlierOpen 1
#define FrontPlierClose 2
#define FrontPlierMiddle 3
#define FrontPlierBalance 4
#define FrontPlierStop 5

// ==============================================================
// 							Definitions
// ==============================================================

typedef struct{
	float stick1_h;
	float stick1_v;
	float stick2_h;
	float stick2_v;
	int intStick_h;
	int intStick_v;
} Sticks;

typedef struct {
	unsigned char square;
	unsigned char cross;
	unsigned char circle;
	unsigned char triangle;
	unsigned char L1;
	unsigned char R1;
	unsigned char L2;
	unsigned char R2;
	unsigned char SE;
	unsigned char ST;
	unsigned char stick1;
	unsigned char stick2;
}Buttons;

typedef struct{
	int mode_type1;
	int mode_type2;
	unsigned char on_state;
}Mode;

typedef struct{
	Sticks sticks;
	Mode modes;
	Buttons but;
}JoyStick;

// ==============================================================
// Global variables
// ==============================================================

int extime = 0;
int button12_prev = 0;
int gripper_last_command = 0;
int frontplier_last_command = 0;
int buttonL1_prev = 0;
float angle = 0.0;

JoyStick controller;




/*	==============================================================
	Definition of ROS CallBack functions
	==============================================================	*/
void joystickCallBack(sensor_msgs::Joy msg){
	// if the ON/OFF button is pressed switch the on_state
	if((unsigned char)msg.buttons[12] == 1 && button12_prev == 0){
		if(controller.modes.on_state){
			controller.modes.on_state = 0;
		}else{
			controller.modes.on_state = 1;
		}
	}button12_prev = (unsigned char)msg.buttons[12];

	// if we can operate
	if(controller.modes.on_state){
			// Change the current values
			controller.but.square = (unsigned char)msg.buttons[0];
			controller.but.cross = (unsigned char)msg.buttons[1];
			controller.but.circle = (unsigned char)msg.buttons[2];
			controller.but.triangle = (unsigned char)msg.buttons[3];
			controller.but.L1 = (unsigned char)msg.buttons[4];
			controller.but.R1 = (unsigned char)msg.buttons[5];
			controller.but.L2 = (unsigned char)msg.buttons[6];
			controller.but.R2 = (unsigned char)msg.buttons[7];
			controller.but.SE = (unsigned char)msg.buttons[8];
			controller.but.ST = (unsigned char)msg.buttons[9];
			controller.but.stick1 = (unsigned char)msg.buttons[10];
			controller.but.stick2 = (unsigned char)msg.buttons[11];
			controller.sticks.stick1_h = -(float)msg.axes[0];
			controller.sticks.stick1_v = (float)msg.axes[1];
			controller.sticks.stick2_h = -(float)msg.axes[2];
			controller.sticks.stick2_v = (float)msg.axes[3];
			controller.sticks.intStick_h = -(int)msg.axes[4];
			controller.sticks.intStick_v = (int)msg.axes[5];
	}
}

/*	==============================================================
								MAIN
	==============================================================	*/

int main(int argc, char **argv){
	// Initialize ROS
	ros::init(argc, argv, "joy_robo"); 	// init the node
	ros::NodeHandle n;					//

	// Definition of the publisher object
	ros::Publisher joystick_publisher = n.advertise<joy_controller::joystick>("JoyStickData",1);
	ros::Publisher point_publisher = n.advertise<geometry_msgs::Point32>("PointCons",1);
	ros::Publisher gripper_publisher = n.advertise<std_msgs::Int16>("GripperCommand",1);
	ros::Publisher frontPlier_publisher = n.advertise<std_msgs::String>("FrontPlierCommand",1);
    ros::Publisher parasol_publisher = n.advertise<std_msgs::Empty>("ParasolCommand",1);

	// Definition of the subscriber object
	ros::Subscriber joystick_sub = n.subscribe("/joy",1,joystickCallBack);

	// Definition of topic variable
	joy_controller::joystick joystick_msg;
	geometry_msgs::Point32 point_msg;
	std_msgs::Int16 gripper_msg;
    std_msgs::String frontplier_msg;
    std_msgs::Empty parasol_msg;

	//sleep(20);

	ros::Rate loop_rate(60); // 50 Hz

	// Clear screen before starting the loop
	printf("\033[2J"); // VT100 escape codes
                       // \033 is the C-style octal code for an escape character
                       // [2J is "Clear entire screen" esc code

	//Initialize States Before Broadcasting
	controller.modes.mode_type1 = 1;
	controller.modes.mode_type2 = 1;

	while(ros::ok()){

		// Wait for the next loop
		ros::spinOnce();
		loop_rate.sleep();
		extime++;

		if(controller.modes.on_state){
            // setup gripper command
			if((controller.but.L2==1)&&(gripper_last_command!=OPEN)){
                gripper_msg.data = 60;
                gripper_last_command = OPEN;
                gripper_publisher.publish(gripper_msg);
            }else if((controller.but.R2==1)&&(gripper_last_command!=CLOSED)){
                gripper_msg.data = 160;
                gripper_last_command = CLOSED;
                gripper_publisher.publish(gripper_msg);
            }

            // setup FrontPlier command
            if((controller.but.square==1)&&(frontplier_last_command!=FrontPlierOpen)){
                frontplier_msg.data = "open";
                frontplier_last_command = FrontPlierOpen;
                frontPlier_publisher.publish(frontplier_msg);
            }else if((controller.but.cross==1)&&(frontplier_last_command!=FrontPlierClose)){
                frontplier_msg.data = "close";
                frontplier_last_command = FrontPlierClose;
                frontPlier_publisher.publish(frontplier_msg);
            }else if((controller.but.circle==1)&&(frontplier_last_command!=FrontPlierMiddle)){
                frontplier_msg.data = "middle";
                frontplier_last_command = FrontPlierMiddle;
                frontPlier_publisher.publish(frontplier_msg);
            }else if((controller.but.triangle==1)&&(frontplier_last_command!=FrontPlierBalance)){
                frontplier_msg.data = "balance";
                frontplier_last_command = FrontPlierBalance;
                frontPlier_publisher.publish(frontplier_msg);
            }else if((controller.but.R1==1)&&(frontplier_last_command!=FrontPlierStop)){
                frontplier_msg.data = "stop";
                frontplier_last_command = FrontPlierStop;
                frontPlier_publisher.publish(frontplier_msg);
            }

            // setup parasol command
            if(controller.but.L1 == 1 && buttonL1_prev== 0){
                parasol_publisher.publish(parasol_msg);
            }buttonL1_prev = controller.but.L1;


            // setup wheel command
			angle -= controller.sticks.stick2_h;
			point_msg.y = angle;
			point_msg.x = 5.0*controller.sticks.stick1_v;
			point_msg.z = 1;
			point_publisher.publish(point_msg);

		}

		// Update and publish messages
		joystick_msg.stick1_h = controller.sticks.stick1_h;
        joystick_msg.stick1_v = controller.sticks.stick1_v;
        joystick_msg.stick2_h = controller.sticks.stick2_h;
        joystick_msg.stick2_v = controller.sticks.stick2_v;
        joystick_msg.intStick_h = controller.sticks.intStick_h;
        joystick_msg.intStick_v = controller.sticks.intStick_v;
        joystick_msg.square = controller.but.square;
        joystick_msg.cross = controller.but.cross;
        joystick_msg.circle = controller.but.circle;
        joystick_msg.triangle = controller.but.triangle;
        joystick_msg.L1 = controller.but.L1;
        joystick_msg.R1 = controller.but.R1;
        joystick_msg.L2 = controller.but.L2;
        joystick_msg.R2 = controller.but.R2;
        joystick_msg.SE = controller.but.SE;
        joystick_msg.ST = controller.but.ST;
        joystick_msg.stick1 = controller.but.stick1;
        joystick_msg.stick2 = controller.but.stick2;
        joystick_msg.mode_type1 = controller.modes.mode_type1;
        joystick_msg.mode_type2 = controller.modes.mode_type2;
        joystick_msg.on_state = controller.modes.on_state;

		joystick_publisher.publish(joystick_msg);

		// Show current states
		printf("\033[5;0HFrom main : The time is: %i\033[K\n", extime);
		printf("\033[6;0HFrom main : The on_state is : %i\033[K\n", controller.modes.on_state);
		printf("\033[7;0HFrom main : The stick 1 is : %f,%f\033[K\n", controller.sticks.stick1_h, controller.sticks.stick1_v);
		printf("\033[8;0HFrom main : The stick 2 is : %f,%f\033[K\n", controller.sticks.stick2_h, controller.sticks.stick2_v);
		printf("\033[9;0HFrom main : The integer stick is : %i,%i\033[K\n", controller.sticks.intStick_h, controller.sticks.intStick_v);
		printf("\033[10;0HFrom main : The states are: type1 '%i', type2 '%i' \033[K\n", controller.modes.mode_type1, controller.modes.mode_type2);
	}

	return EXIT_SUCCESS;

}
