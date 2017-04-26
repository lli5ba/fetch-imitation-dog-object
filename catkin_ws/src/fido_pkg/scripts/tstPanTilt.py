#!/usr/bin/env python

## Based on wander.py from Programming Robots with ROS
#
# http://proquest.safaribooksonline.com/book/hardware/9781449325480/3dot-topics/publishing_html#X2ludGVybmFsX0h0bWxWaWV3P3htbGlkPTk3ODE0NDkzMjU0ODAlMkZpZHAyMjI1MTMyOF9odG1sJnF1ZXJ5PQ==

import rospy
from fido_pkg.msg import PanTilt
from fido_pkg.msg import BallPixel

def processPanTilt(data): 
    global panTilt
    panTilt['pan'] = data.pan
    panTilt['tilt'] = data.tilt

    rospy.loginfo("Pan/Tilt Event: Pan Angle="+str(data.pan)+"  Tilt Angle="+str(data.tilt))

    
panTilt = {}
panTilt['pan'] = 0
panTilt['tilt'] = 0

rospy.Subscriber('fido/PanTilt', PanTilt, processPanTilt)
ball_loc_pub = rospy.Publisher('/fido/ballLoc', BallPixel, queue_size=1)
rospy.init_node('tstPanTilt')
state_change_time = rospy.Time.now()

rate = rospy.Rate(10)

def shutdown():
    rospy.loginfo("Stop!") 
    ball_loc_pub.publish(BallPixel()) 
    rospy.sleep(1)
    
rospy.on_shutdown(shutdown)

totalSteps = 6
steps = [ BallPixel() for i in range(totalSteps)]

steps[0].x = 0
steps[0].y = 0

steps[1].x = 90
steps[1].y = 0

steps[2].x = -90
steps[2].y = 0

steps[3].x = 0
steps[3].y = 100

steps[4].x = 0
steps[4].y = -30

steps[5].x = 0
steps[5].y = 0


stepNum = 0
state_change_time = rospy.Time.now()
while not rospy.is_shutdown():

    ## if rospy.Time.now() > state_change_time:
    ##   state_change_time = rospy.Time.now() + rospy.Duration(2)
    ##   driving = turn
    ## else: # we're not driving_forward
    ##     if rospy.Time.now() > state_change_time:
    ##         driving = Dforward # we're done spinning, time to go forward!

    if (rospy.Time.now() > state_change_time) and (stepNum < totalSteps):
        state_change_time = rospy.Time.now() + rospy.Duration(1.5)

        bp = steps[stepNum]
        ball_loc_pub.publish(bp)
        stepNum += 1
        
    rate.sleep()
  
