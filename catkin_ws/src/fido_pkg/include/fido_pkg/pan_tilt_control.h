#ifndef PAN_TILT_CONTROL_H
#define PAN_TILT_CONTROL_H

#include <ros/ros.h>
#include <math.h>

//message headers
#include <fido_pkg/BallPixel.h>
#include <fido_pkg/PanTilt.h>

// Operator Overloading for PanTilt structures
//
// Overload + operator to add two PanTilt objects.
inline fido_pkg::PanTilt operator+(const fido_pkg::PanTilt& a, const fido_pkg::PanTilt& b) {
    fido_pkg::PanTilt pt;

    pt.pan = a.pan + b.pan;
    pt.tilt = a.tilt + b.tilt;

    return pt;
}

// Overload - operator to subtract two PanTilt objects.
inline fido_pkg::PanTilt operator-(const fido_pkg::PanTilt& a, const fido_pkg::PanTilt& b) {
    fido_pkg::PanTilt pt;

    pt.pan = a.pan - b.pan;
    pt.tilt = a.tilt - b.tilt;

    return pt;
}

// Overload * operator to multiply two PanTilt objects.
inline fido_pkg::PanTilt operator*(const fido_pkg::PanTilt& a, const fido_pkg::PanTilt& b) {
    fido_pkg::PanTilt pt;

    pt.pan = a.pan * b.pan;
    pt.tilt = a.tilt * b.tilt;

    return pt;
}

// Overload * operator to multiply PanTilt object with a float.
inline fido_pkg::PanTilt operator*(const fido_pkg::PanTilt& a, const float b) {
    fido_pkg::PanTilt pt;

    pt.pan = (int) round(a.pan * b);
    pt.tilt = (int) round(a.tilt * b);

    return pt;
}

// Add an int16 value
inline fido_pkg::PanTilt operator+(const fido_pkg::PanTilt& a, const int16_t b) {
    fido_pkg::PanTilt pt;

    pt.pan = a.pan + b;
    pt.tilt = a.tilt + b;

    return pt;
}

// divide by an int16 value
inline fido_pkg::PanTilt operator/(const fido_pkg::PanTilt& a, const int16_t b) {
    fido_pkg::PanTilt pt;

    pt.pan = a.pan / b;
    pt.tilt = a.tilt / b;

    return pt;
}


// PanTilt is defined so that zero is assigned the values 0 & 0 for pan and tilt
const fido_pkg::PanTilt zero;

class Pan_Tilt_Controller
{
 private:
    //@@@ros::NodeHandle nh_;
 
    ros::Subscriber ptAngle_subscriber;

    ros::Publisher pan_tilt_control;
    fido_pkg::PanTilt ball_pose;
    fido_pkg::PanTilt current_pose;

    fido_pkg::PanTilt ball_velo; 
    fido_pkg::PanTilt camera_velo; 

    bool movingCamera;
  
    float servo_K_x;
    float servo_K_y;
    int camera_F_x;
    int camera_F_y;

    fido_pkg::PanTilt screen;	/* screen width and height in servo pan & tilt angles */
    fido_pkg::PanTilt min_angle_diff;
    fido_pkg::PanTilt angle_goal_post_min;
    fido_pkg::PanTilt angle_goal_post_max;
    fido_pkg::PanTilt angle_max;
    fido_pkg::PanTilt angle_min;
 
 public:
    Pan_Tilt_Controller(ros::NodeHandle *nh, const int screenmaxx=640, const int screenmaxy=480);
    ~Pan_Tilt_Controller();

    bool isCameraMoving(){return movingCamera;};

    int ball_angle(const int cameraAngle, const int error, const int focal);
    void track_ball(const double timeStep, const fido_pkg::BallPixel& ballError, fido_pkg::PanTilt& panTiltAngles );
    void panTiltAnglecallback(const fido_pkg::PanTilt::ConstPtr& msg);

};
 
#endif
