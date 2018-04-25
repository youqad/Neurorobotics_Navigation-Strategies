#!/usr/bin/env python

import rospy
from subsomption.msg import Channel 
from std_msgs.msg import Bool,Int16MultiArray
from math import *
from sensor_msgs.msg import LaserScan

import random

bumper_l=0
bumper_r=0
radars=[]#Int16MultiArray()
lasers=LaserScan() # used to avoid getting stuck in obstacles not detected by the bumpers

def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)

def callback_radars(data):
    global radars
    radars=data.data
    #rospy.loginfo(rospy.get_caller_id()+" Radars "+str(radars))

def callback_lasers(data):
    global lasers
    lasers=data
    #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# Behavior based on a radar sensor, where the robots turns towars the quadrant where the target is detected
def radarGuidance():
    rospy.init_node('radarGuidance', anonymous=True)

    # remove the subscriptions you don't need.
    rospy.Subscriber("/simu_fastsim/radars", Int16MultiArray, callback_radars)
    rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)

    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
    pub = rospy.Publisher('/navigation_strategies/channel1', Channel , queue_size=10)
    r = rospy.Rate(10) # 10hz

    # behavioral parameters:
    v_fwd = 1.5
    v_turn = 0.75

    v=Channel()
    v.activated=False
    v.speed_left = 0
    v.speed_right = 0

    # scans used to check wall distances:
    angleLMin = 0
    angleLMax = 55

    angleFMin = 56
    angleFMax = 143

    angleRMin = 144 #199-55
    angleRMax = 199

    th_obstacleTooClose = 13


    while not rospy.is_shutdown():
      # compute the value of v that will be sent to the subsomption channel. 

      wallTooCloseL = False
      wallTooCloseF = False
      wallTooCloseR = False

      v.speed_left = 0
      v.speed_right = 0

      # check if we are really receiving laser scan measurements in accordance with the robot settings
      if len(lasers.ranges) == 200:
        # determine if obstacle too close:
        for i in range(len(lasers.ranges)):
          #rospy.loginfo("front:"+str(l))
          if lasers.ranges[i] < th_obstacleTooClose:
            if i in range(angleLMin,angleLMax):
              wallTooCloseL = True
            if i in range(angleFMin,angleFMax):
              wallTooCloseF = True
            if i in range(angleRMin,angleRMax):
              wallTooCloseR = True


      # check if we are receiving radar measurements
      if radars != 0:
        radars_list = []
        for i in range(len(radars)):
          radars_list.append(radars[i])
        #rospy.loginfo(str(radars_list))

        # if the goal i in front of the robot :
        if wallTooCloseF:
          rospy.loginfo('WALL F')
          v.speed_left =  -v_fwd
          v.speed_right = -v_fwd
        elif bumper_r or wallTooCloseR:
          rospy.loginfo('WALL R')
          v.speed_left =  v_fwd
          v.speed_right = -v_fwd
        elif bumper_l or wallTooCloseL:
          rospy.loginfo('WALL L')
          v.speed_left =  -v_fwd
          v.speed_right = v_fwd 
        elif (7 in radars_list) :
          rospy.loginfo('FWD L')
          v.speed_left =  v_fwd
          v.speed_right = v_fwd*.95
        elif (0 in radars_list):
          rospy.loginfo('FWD R')
          v.speed_left =  v_fwd*.95
          v.speed_right = v_fwd
        # if it is on the left :
        elif (6 in radars_list) or (5 in radars_list):
          rospy.loginfo('LEFT')
          v.speed_left =  v_fwd
          v.speed_right = v_turn
        # if it is on the right :
        elif (1 in radars_list) or (2 in radars_list):
          rospy.loginfo('RIGHT')
          v.speed_left =  v_turn
          v.speed_right = v_fwd
        # if it is  behind :
        elif (3 in radars_list) :
          rospy.loginfo('BEHIND R')
          v.speed_left =  -v_fwd
          v.speed_right = v_fwd
        elif (4 in radars_list) :
          rospy.loginfo('BEHIND L')
          v.speed_left =  v_fwd
          v.speed_right = -v_fwd
        # publish the suggested movement
        pub.publish(v)
 
      r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        radarGuidance()
    except rospy.ROSInterruptException: pass
