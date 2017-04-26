#include <math.h>

//ROS headers
#include <ros/ros.h>

//message headers
#include <fido_pkg/BallPixel.h>
#include <fido_pkg/PanTilt.h>
//#include <fido_pkg/Robot_Control.h>

// Class Header
#include <fido_pkg/robot_control.h>

Robot_Controller::Robot_Controller (ros::NodeHandle *nh)
{
    //Loading Default values
    //

    //Accessing parameters from *.yaml used by instatiator 
    //
    // Due to bug in ROS Indigo, cannot use floats or doubles in
    // parameter file, so use int's and then divide by 1000 to form
    // the float's that we want.
    int k1_init_int=1000,k2_init_int=1000,k3_init_int=1000;
    int cameraHeight_init_int = 530; //53 cm, approx
    int s_init_int = 160; //16cm, approx
    
    
    try{
	nh->getParam("Kr_1", k1_init_int);
	nh->getParam("Kr_2", k2_init_int);
	nh->getParam("Kr_3", k3_init_int);
	nh->getParam("robot_s", s_init_int);
	nh->getParam("robot_h", cameraHeight_init_int);

	ROS_INFO("Successfully Loaded robot_control parameters");
    }

    catch(int e)
	{
	    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
	}
   

    k1_init = (float) k1_init_int / 1000;
    k2_init = (float) k2_init_int / 1000;
    k3_init = (float) k3_init_int / 1000;
    cameraHeight_init = (float) cameraHeight_init_int / 1000;
    s_init = (float) s_init_int / 1000;

    //TODO: Create message type  (at least I think that's what's in the angle brackets)
    //robot_control = nh->advertise<fido_pkg::Robot_Control>("/fido/robot_velocities",1);
    //offsetAngle_subscriber = nh->subscribe("/fido/PanTiltArduRead",1,&Pan_Tilt_Controller::newOffsetAngles, this);

}

Robot_Controller::~Robot_Controller()
{
}
  

void Robot_Controller::update(const double timeStep, const fido_pkg::BallPixel& ballError, const fido_pkg::PanTilt& panTiltAngles)
{


    if (ballError.ballChange) {
    }


    //ROS_INFO("Robot Control: timeStep = %f, error_x = %d, error_y = %d", timeStep, ballError.x, ballError.y);
    //ROS_INFO("  pan (deg) = %d, tilt (deg) = %d", panTiltAngles.pan, panTiltAngles.tilt); 

    //pan_tilt_control.publish(current_pose);


}

void Robot_Controller::newPTAngles()
{
}

void Robot_Controller::newOffsetAngles()
{
}

void Robot_Controller::send()
{
}

