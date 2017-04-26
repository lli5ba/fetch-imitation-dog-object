/*
 * Copyright (C) 2017, Lentin Joseph and Qbotics Labs Inc.

 * Email id : qboticslabs@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.


*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>

#include "fido_pkg/BallPixel.h"
#include "fido_pkg/PanTilt.h"

#define PI (3.14159265)

//Tracker parameters

//@@@int servomaxx, servomin,screenmaxx, center_offset, center_left, center_right;
// int servomaxx, servomin, center_offset;
// int screenmaxx, screenmaxy;

float servo_K_x;
float servo_K_y;
int camera_F_x;
int camera_F_y;

fido_pkg::PanTilt ball_pose;
fido_pkg::PanTilt current_pose;

double current_time;

//@@@ros::Publisher dynamixel_control;
ros::Publisher pan_tilt_control;

bool firstTime;
bool movingCamera;

//@@@int servo_step_distancex, current_pos_x;
// void track_face_OLD(int x,int y)
// {
//     int error_x;

//     //Find out if the X component of the face is to the left of the middle of the screen.
//     if(x < (center_left)){

// 	ROS_INFO("Face is at Left");
	
// 	// Determine error from center
// 	error_x = center_left - x;

// 	current_pos_x += (error_x / servo_K_x);
// 	current_pose.x = current_pos_x;
// 	ROS_INFO("servo controller current_position = %d",current_pos_x);

//         //if(current_pos_x < servomaxx and current_pos_x > servomin ){
//         //@@@dynamixel_control.publish(current_pose);
// 	//}
// 	ball_loc_control.publish(current_pose);
//     }

    
//     //Find out if the X component of the face is to the right of the middle of the screen.
//     else if(x > center_right){

// 	ROS_INFO("Face is at Right");

// 	// Determine error from center
// 	error_x = x - center_right;

// 	current_pos_x -= (error_x / servo_K_x);
// 	current_pose.x = current_pos_x;

// 	ROS_INFO("servo controller current_position = %d",current_pos_x);

// 	//if(current_pos_x < servomaxx and current_pos_x > servomin ){
//         	//@@@dynamixel_control.publish(current_pose);
// 	//}
// 	ball_loc_control.publish(current_pose);

//     }

//    else if(x > center_left and x < center_right){

// 	ROS_INFO("Face is at Center");
        
// 	}


// }

// This function returns the angle of the ball w.r.t the robot center
int ball_horizontal_angle(const int pan, const int error_x, const int focal_x)
{
    return pan + (int) round(atan2(-error_x, focal_x) * (180/PI));
}

void track_ball(int x,int y,bool ballChange)
{
    int error_x;
    int error_y;
    fido_pkg::PanTilt ball_diff;
    fido_pkg::PanTilt prop_pose;
    fido_pkg::PanTilt delta_pose;

    // Determine X error from center
    //@@@@error_x = (screenmaxx / 2) - x;
    error_x = x;		// error is passed in
    
    // The below control law from the paper is not quite working, so go back to proportional control 
    //@@@current_pose.pan -= (int) (servo_K_x * error_x);

    double timeStep; // time since last call of track_ball

    // current_pose.pan = current_pose.pan - ( ball_diff.pan + (int) (timeStep * servo_K_x * error_x) );
    // ball_pose.pan = ball_diff.pan + ball_pose.pan;

    timeStep = ros::Time::now().toSec() - current_time;
    current_time += timeStep; 	// update current_time
    
    if (ballChange) {
	// If a change in targets, zero out ball_diff by making ball_pose ball_horizontal_angle()
	// ball_pose will become the angle w.r.t the camera angles.
	ball_pose.pan = ball_horizontal_angle(current_pose.pan, error_x, camera_F_x);
	ball_pose.tilt = 0;
    }

    // @@@@ Tests  
    //
    // NOTE: If error_x is negative, it means that target is on the
    // left half of the image plane. That requires a positive angle
    // pan to move to the left. So negate error_x to get the servo pan
    // direction correct.

    ball_diff.pan = ball_horizontal_angle(current_pose.pan, error_x, camera_F_x) - ball_pose.pan;
    ball_pose.pan = ball_diff.pan + ball_pose.pan; // update ball pose for next iteration
    
    prop_pose.pan = (int) round(timeStep * servo_K_x * -error_x);
    delta_pose.pan = ball_diff.pan + prop_pose.pan;

    // Determine Y error from center
    //@@@@error_y = (screenmaxy / 2) - y;
    error_y = y;		// error is passed in
    delta_pose.tilt = (int) (servo_K_y * error_y);

    current_pose.pan += delta_pose.pan;
    //@@@@current_pose.pan += ball_horizontal_angle(0, error_x, camera_F_x); //@@@@+ prop_pose.pan;
    current_pose.tilt += delta_pose.tilt;

    ROS_INFO("timeStep = %f, servo_K_x = %f, error_x = %d, P = %d", timeStep, servo_K_x, error_x, prop_pose.pan);
    ROS_INFO("  ball_pose.pan = %d, ball_diff.pan = %d, camera_F_x = %d", ball_pose.pan, ball_diff.pan, camera_F_x);
    ROS_INFO("  current_pose = %d (pan), %d (tilt) [ball_diff: %d (pan)]",current_pose.pan,current_pose.tilt,ball_diff.pan);

    if (delta_pose.pan != 0 || delta_pose.tilt != 0) {
	// Only publish new pan/tilt angles if there is a change in angles
	movingCamera = true;	// indicate that camera is moving
	pan_tilt_control.publish(current_pose);
    }

}

//Callback of the topic /numbers
void ballErrorcallback(const fido_pkg::BallPixel::ConstPtr& msg)
{
	//ROS_INFO("Recieved X = [%d], Y = [%d]",msg->x,msg->y);

    if (firstTime) {
	// If first call since started, init the time so can know the
	// time between steps. This will mean timeStep of the first
	// call will be near 0, but that is okay.
	current_time = ros::Time::now().toSec();
	firstTime = false;	// clear first time flag
    }
	
    if (!movingCamera) {
	// Calling track ball function, if we think that the camera is
	// not moving (otherwise, do not know the true angle of the
	// camera when this msg was sent - this can throw off
	// calculations by quite a bit).
	//
	// msg->x is the horizontal error from desired location (typically center) in pixels
	// msg->y is the vertical error from desired location (typically center) in pixels
	track_ball(msg->x,msg->y,msg->ballChange);
    }

}

//Callback of the topic /numbers
void panTiltAnglecallback(const fido_pkg::PanTilt::ConstPtr& msg)
{
    current_pose = *msg;

    ROS_INFO("Current Pan/Tilt Angles Pan = [%d], Tilt = [%d]",current_pose.pan,current_pose.tilt);	
    ros::Duration(0.6).sleep(); // sleep for 1 second (@@@@ as a test)
    movingCamera = false;	// indicate that the camera has stopped moving
}

int main(int argc, char **argv)
{

int servo_K_x_int;
int servo_K_y_int;

	//Loading servo configurations of the dynamixel tracker
	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"pan_tilt_controller");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber number_subscriber = node_obj.subscribe("/fido/BallError",1,ballErrorcallback);
        //@@@dynamixel_control = node_obj.advertise<std_msgs::Float64>("/pan_controller/command",10);
	pan_tilt_control = node_obj.advertise<fido_pkg::PanTilt>("/fido/PanTiltArduGo",1);
	ros::Subscriber ptAngle_subscriber = node_obj.subscribe("/fido/PanTiltArduRead",1,panTiltAnglecallback);

	//@@@
	// servomaxx = 90;   //max degree servo horizontal (x) can turn
	// servomin = -90;
	// screenmaxx = 640;   //max screen horizontal (x)resolution
	// screenmaxy = 480;   //max screen vertical (y)resolution
	// center_offset = 100;
	//@@@servo_step_distancex = 1; //x servo rotation steps
	servo_K_x_int = 200;
	servo_K_y_int = 100;
	//current_pos_x =0 ;
	camera_F_x = 546;
	camera_F_y = 546;

	  try{
	      //@@@
	      // node_obj.getParam("servomaxx", servomaxx);
	      // node_obj.getParam("servomin", servomin);
	      // node_obj.getParam("screenmaxx", screenmaxx);
	      // node_obj.getParam("screenmaxy", screenmaxy);
	      // node_obj.getParam("center_offset", center_offset);
	      //@@@node_obj.getParam("step_distancex", servo_step_distancex);
	      node_obj.getParam("K_x", servo_K_x_int);
	      node_obj.getParam("K_y", servo_K_y_int);
	      node_obj.getParam("F_x", camera_F_x);
	      node_obj.getParam("F_y", camera_F_y);

	      ROS_INFO("Successfully Loaded tracking parameters");
	  }


	  catch(int e)
	  {
	   
	      ROS_WARN("Parameters are not properly loaded from file, loading defaults");
	
	  }
	  //center_left = (screenmaxx / 2) - center_offset;
	  //center_right = (screenmaxx / 2) + center_offset;

	  servo_K_x = (float) servo_K_x_int / 1000;
	  servo_K_y = (float) servo_K_y_int / 1000;


	//Sending initial pose
	current_pose.pan = 0;
	current_pose.tilt = 0;
        //@@@dynamixel_control.publish(initial_pose);
	pan_tilt_control.publish(current_pose);

	ball_pose.pan = 0;
	ball_pose.tilt = 0;

	// Init the time so can know the time between steps
	current_time = ros::Time::now().toSec();
	firstTime = true;
	movingCamera = false;	// camera should not be moving at the start

	//Spinning the node
	ros::spin();
	return 0;
}


