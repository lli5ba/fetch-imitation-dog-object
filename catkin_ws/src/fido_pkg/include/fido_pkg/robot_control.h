#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <ros/ros.h>
#include <math.h>

//message headers
#include <fido_pkg/BallPixel.h>
#include <fido_pkg/PanTilt.h>

class Robot_Controller
{
 private:
    //ros::Subscriber ptAngle_subscriber;
    //ros::Subscriber offsetAngle_subscriber;

    //ros::Publisher robot_control; //Publishing to robot motors, might remove.

    float k1_init,k2_init,k3_init;
    float cameraHeight_init;
    float s_init;

    float k1;
    float k2;
    float k3;
    float errorx;
    float errory;
    float vbx;
    float vby;
    float pan;
    float tilt;
    float s;
    float ballLocx;
    float ballLocy;
    float cameraAngleToBall;
    float robotAngleToBall;
    float cameraHeight;

 
 public:
    Robot_Controller(ros::NodeHandle *nh);
    ~Robot_Controller();
    
    void update(const double timeStep, const fido_pkg::BallPixel& ballError, const fido_pkg::PanTilt& panTiltAngles);
    void newPTAngles();
    void newOffsetAngles();
    void send();
    //bool isCameraMoving(){return movingCamera;};

    //int ball_angle(const int cameraAngle, const int error, const int focal);
    //void track_ball(const fido_pkg::BallPixel& ballError);
    //void panTiltAnglecallback(const fido_pkg::PanTilt::ConstPtr& msg);

};
 
#endif
