

#include <math.h>

//ROS headers
#include <ros/ros.h>

//message headers
#include <fido_pkg/BallPixel.h>
#include <fido_pkg/PanTilt.h>

// Class Header
#include <fido_pkg/pan_tilt_control.h>

Pan_Tilt_Controller::Pan_Tilt_Controller (ros::NodeHandle *nh, const int screenmaxx, const int screenmaxy)
{
    //Loading Default values
    movingCamera = false;

    //Accessing parameters from *.yaml used by instatiator
    int servo_K_x_int;
    int servo_K_y_int;
    int target_thresh_percent_x;
    int target_thresh_percent_y;

    servo_K_x_int = 200;
    servo_K_y_int = 100;
    camera_F_x = 546;
    camera_F_y = 546;
    target_thresh_percent_x = 5;   // make it 5% of screen 
    target_thresh_percent_y = 5;   // make it 5% of screen 

    int angle_max_pan;
    int angle_min_pan;
    int angle_max_tilt;
    int angle_min_tilt;
 
    angle_max_pan = 85;
    angle_min_pan = -85;
    angle_max_tilt = 30;
    angle_min_tilt = -95;

    try{
	//nh_.getParam("screenmaxx", screenmaxx);
	//nh_.getParam("screenmaxy", screenmaxy);
	nh->getParam("K_x", servo_K_x_int);
	nh->getParam("K_y", servo_K_y_int);
	nh->getParam("F_x", camera_F_x);
	nh->getParam("F_y", camera_F_y);
	nh->getParam("targetThresholdPrct_x", target_thresh_percent_x);
	nh->getParam("targetThresholdPrct_y", target_thresh_percent_y);
	nh->getParam("angle_max_pan", angle_max_pan);
	nh->getParam("angle_min_pan", angle_min_pan);
	nh->getParam("angle_max_tilt", angle_max_tilt);
	nh->getParam("angle_min_tilt", angle_min_tilt);

	ROS_INFO("Successfully Loaded pan_tilt_control parameters");
    }

    catch(int e)
	{
   
	    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
	
	}
   
    pan_tilt_control = nh->advertise<fido_pkg::PanTilt>("/fido/PanTiltArduGo",1);
    ptAngle_subscriber = nh->subscribe("/fido/PanTiltArduRead",1,&Pan_Tilt_Controller::panTiltAnglecallback, this);

    //Sending initial pose
    current_pose = zero;
    pan_tilt_control.publish(current_pose);

    angle_goal_post_min = zero;
    angle_goal_post_max = zero;

    angle_max.pan = angle_max_pan;
    angle_max.tilt = angle_max_tilt;
    angle_min.pan = angle_min_pan;
    angle_min.tilt = angle_min_tilt;

    ball_pose = zero;

    fido_pkg::PanTilt target_thresh_percent;
    target_thresh_percent.pan = target_thresh_percent_x;
    target_thresh_percent.tilt = target_thresh_percent_y;

    // clear ball and camera velocities
    ball_velo = zero;
    camera_velo = zero;

    servo_K_x = (float) servo_K_x_int / 1000;
    servo_K_y = (float) servo_K_y_int / 1000;

    // compute the screen width and height in pan and tilt angles 
    //
    // ball_angle() expects the 2nd and 3rd parameter to describe
    // lengths of opposite and adjacent sides of a right
    // triangle. Therefore, when determining the servo angles for the
    // entire width, need to make opposite cut in half to be a right
    // angle and then multiply the ball_angle() by 2.
    screen.pan = ball_angle(0, screenmaxx/2, camera_F_x) * 2;
    screen.tilt = ball_angle(0, screenmaxy/2, camera_F_y) * 2;

    min_angle_diff = (screen * target_thresh_percent) / 100;

    ROS_INFO("servo K_x = %f, servo K_y = %f", servo_K_x, servo_K_y);
    ROS_INFO("Servo Angle Min Thresholds: %d (pan) %d (tilt)", min_angle_diff.pan, min_angle_diff.tilt);

    

}

Pan_Tilt_Controller::~Pan_Tilt_Controller()
{
}
  
// This function returns the angle of the ball w.r.t the robot center
int Pan_Tilt_Controller::ball_angle(const int cameraAngle, const int error, const int focal)
{
    return cameraAngle + (int) round(atan2(error, focal) * (180/M_PI));
}

void Pan_Tilt_Controller::track_ball(const double timeStep, const fido_pkg::BallPixel& ballError, fido_pkg::PanTilt& panTiltAngles )
{
    int error_x;
    int error_y;
    fido_pkg::PanTilt ball_diff;
    fido_pkg::PanTilt prop_pose;
    fido_pkg::PanTilt delta_pose;

    fido_pkg::PanTilt prev_camera;
    fido_pkg::PanTilt prev_ball;

    fido_pkg::PanTilt new_camera;

    // Determine X error from center
    //@@@@error_x = (screenmaxx / 2) - x;
    error_x = ballError.x;		// error is passed in
    
    // Determine Y error from center
    //@@@@error_y = (screenmaxy / 2) - y;
    error_y = ballError.y;		// error is passed in
    //@@@delta_pose.tilt = (int) (servo_K_y * error_y);


    // The below control law from the paper is not quite working, so go back to proportional control 
    //@@@current_pose.pan -= (int) (servo_K_x * error_x);

    // current_pose.pan = current_pose.pan - ( ball_diff.pan + (int) (timeStep * servo_K_x * error_x) );
    // ball_pose.pan = ball_diff.pan + ball_pose.pan;

    prev_ball = ball_pose;
    prev_camera = current_pose;

    if (ballError.ballLost) {
	// Did not find a ball in this last video frame, so based on past ball velocity, guess where the ball is
	if (prev_camera.pan > (angle_max.pan - min_angle_diff.pan) || prev_camera.pan < (angle_min.pan + min_angle_diff.pan) 
	    || prev_camera.tilt > (angle_max.tilt - min_angle_diff.tilt) || prev_camera.tilt < (angle_min.tilt + min_angle_diff.tilt)) {
	    // cannot continue to turn (either pan or tilt or both) so consider the ball lost and go back to the home position
	    prev_camera = zero;
	    delta_pose = zero;

	    // Also, zero out ball velocity so camera stops trying to seek the ball
	    ball_velo = zero;
	} else {
	    // use last known ball pan velocity to guess where it is and move camera at same velocity
	    //@@@delta_pose = ball_velo * timeStep;

	    // move camera as fast as possible based on the total angles in the camera "screen"
	    //@@@delta_pose = screen / 2;

	    // use last known ball pan velocity to guess where it is and move camera at 2x velocity
	    delta_pose = (ball_velo * 2) * timeStep;

	}

	new_camera = prev_camera + delta_pose;

	// define it as something for below ROS_INFO
	ball_diff = zero;

	ROS_INFO("PTC: Ball Lost!");

    } else {

	if (ballError.ballChange) {
	    // If a change in targets, zero out ball_diff by making ball_pose ball_angle()
	    // ball_pose will become the angle w.r.t the camera angles.
	    ball_pose.pan = ball_angle(prev_camera.pan, -error_x, camera_F_x);
	    ball_pose.tilt = ball_angle(prev_camera.tilt, error_y, camera_F_y);
	}

	// NOTE: If error_x is negative, it means that target is on the
	// left half of the image plane. That requires a positive angle
	// pan to move to the left. So negate error_x to get the servo pan
	// direction correct.


	ball_diff.pan = ball_angle(prev_camera.pan, -error_x, camera_F_x) - ball_pose.pan;
	ball_diff.tilt = ball_angle(prev_camera.tilt, error_y, camera_F_y) - ball_pose.tilt;

	// update ball pose for next iteration
	ball_pose = ball_diff + ball_pose;
    
	prop_pose.pan = (int) round(timeStep * servo_K_x * -error_x);
	prop_pose.tilt = (int) round(timeStep * servo_K_y * error_y);

	delta_pose = ball_diff + prop_pose;
	new_camera = prev_camera + delta_pose;

	//new_camera.pan += delta_pose.pan;
	//@@@@new_camera.pan += ball_angle(0, -error_x, camera_F_x); //@@@@+ prop_pose.pan;
	//new_camera.tilt += delta_pose.tilt;

	// for debug, compute camera and ball velocity
	ball_velo.pan = timeStep == 0 ? 0 : (int) round((double) ball_diff.pan / timeStep);
	ball_velo.tilt = timeStep == 0 ? 0 : (int) round((double) ball_diff.tilt / timeStep);
	camera_velo.pan = timeStep == 0 ? 0 : (int) round((double) delta_pose.pan / timeStep);
	camera_velo.tilt = timeStep == 0 ? 0 : (int) round((double) delta_pose.tilt / timeStep);

    }

    if (ballError.ballChange) {
	ROS_INFO("Ball Change!");
	ROS_INFO("  timeStep = %f, error_x = %d, P = %d", timeStep, error_x, prop_pose.pan);
	ROS_INFO("  current_pose = %d (pan), %d (tilt)",current_pose.pan,current_pose.tilt);
	ROS_INFO("  ang. velocity (degrees/s): ball = %d (pan) / %d (tilt), camera =  %d (pan) / %d (tilt)", 
		 ball_velo.pan, ball_velo.tilt, camera_velo.pan, camera_velo.tilt);
    }

    if (new_camera.pan < angle_goal_post_min.pan || new_camera.pan > angle_goal_post_max.pan 
	|| new_camera.tilt < angle_goal_post_min.tilt || new_camera.tilt > angle_goal_post_max.tilt) {
	// Only publish new pan/tilt angles if there is a significant change in angles
	movingCamera = true;	// indicate that camera is moving

	ROS_INFO("timeStep = %f, servo_K (x/y) = %f/%f, error_x = %d, P = %d", timeStep, servo_K_x, servo_K_y, error_x, prop_pose.pan);
	ROS_INFO("  prev_ball.pan = %d, prev_ball.tilt = %d, prev_camera.pan = %d, prev_camera.tilt = %d", 
		 prev_ball.pan, prev_ball.tilt, prev_camera.pan, prev_camera.tilt);
	ROS_INFO("  ball_pose.pan = %d, ball_diff.pan = %d, camera_F (x/y) = %d/%d", ball_pose.pan, ball_diff.pan, camera_F_x, camera_F_y);
	ROS_INFO("  delta_pose = %d (pan), %d (tilt)",delta_pose.pan,delta_pose.tilt);
	ROS_INFO("  new_camera = %d (pan), %d (tilt)",new_camera.pan,new_camera.tilt);
	ROS_INFO("  ang. velocity (degrees/s): ball = %d (pan) / %d (tilt), camera =  %d (pan) / %d (tilt)", 
		 ball_velo.pan, ball_velo.tilt, camera_velo.pan, camera_velo.tilt);

	current_pose = new_camera;
	pan_tilt_control.publish(current_pose);

	// update goal posts for next time
	angle_goal_post_min = current_pose - min_angle_diff;
	angle_goal_post_max = current_pose + min_angle_diff;
	ROS_INFO("  pan goal posts: %d / %d, tilt goal posts: %d / %d", 
		 angle_goal_post_min.pan, angle_goal_post_max.pan, 
		 angle_goal_post_min.tilt, angle_goal_post_max.tilt);

    }

    // Return the updated values for the Pan and Tilt Angles
    panTiltAngles = current_pose;

}

//Callback of the topic /numbers
void Pan_Tilt_Controller::panTiltAnglecallback(const fido_pkg::PanTilt::ConstPtr& msg)
{
    current_pose = *msg;

    ROS_INFO("Current Pan/Tilt Angles Pan = [%d], Tilt = [%d]",current_pose.pan,current_pose.tilt);	
    ros::Duration(0.35).sleep(); // wait just a little bit longer
    movingCamera = false;	// indicate that the camera has stopped moving
}


