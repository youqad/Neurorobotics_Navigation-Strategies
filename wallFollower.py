#!/usr/bin/env python

import rospy
from subsomption.msg import Channel 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from math import *

import random

bumper_l=0
bumper_r=0
lasers=LaserScan()

def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)


def callback_lasers(data):
    global lasers
    lasers=data
    #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# Behavior based on the laser scanner, where the robots tries to keep obstacles in an interval band on the right or on the left
def wallFollower():
    rospy.init_node('wallFollower', anonymous=True)

    # remove the subscriptions you don't need.
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
    rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
 
    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
    pub = rospy.Publisher('/navigation_strategies/channel0', Channel , queue_size=10)
    r = rospy.Rate(10) # 10hz

    # behavioral parameters:
    lMaxRange = 3000
    robotRadius = 10 #15
    th_obstacleFront = 30 #35
    th_wallTooClose = 20 #25
    th_wallTooFar = 30 #35
    th_neglectedWall = 40 #50
    v_fwd = 1
    v_turn = 0.9
    v_turnf = 0.5

    v=Channel()
    v.activated=False
    v.speed_left = 0
    v.speed_right = 0

    # computation of the scans used to check front obstacle:
    angleFrontMin = 99 - int(atan2(robotRadius,th_obstacleFront)/pi*180)
    angleFrontMax = 100 + int(atan2(robotRadius,th_obstacleFront)/pi*180)

    # scans used to check wall distances:
    angleLMin = 0
    angleLMax = 55

    angleRMin = 199-55
    angleRMax = 199

    rospy.loginfo("Wall Follower: Angles "+str(angleFrontMin)+" "+str(angleFrontMax)+" "+str(angleLMin)+" "+str(angleLMax)+" "+str(angleRMin)+" "+str(angleRMax))

    lastWallOnLeft = True

    while not rospy.is_shutdown():
      # compute the value of v that will be sent to the subsomption channel. 
      v.speed_left = 0
      v.speed_right = 0

      obstacleFront = False
      wallTooCloseL = False
      wallTooFarL = False
      wallOKL = False
      wallOKR = False
      wallTooCloseR = False
      wallTooFarR = False

      distFrontMin = lMaxRange

      # check if we are really receiving laser scan measurements in accordance with the robot settings
      if len(lasers.ranges) == 200:
        # determine if obstacle in front:
        for l in lasers.ranges[angleFrontMin:angleFrontMax]:
          #rospy.loginfo("front:"+str(l))
          if l < distFrontMin:
            distFrontMin = l
          if l < th_obstacleFront:
            obstacleFront = True

        # determine if walls are within the "too close" and the "too far" L & R bands:
        distWallLMin = lMaxRange
        distWallRMin = lMaxRange

        for i in range(angleLMin,angleLMax):
          if lasers.ranges[i] < distWallLMin:
            distWallLMin = lasers.ranges[i]
          if lasers.ranges[i]*cos((10-i)/180.*pi) < th_wallTooClose:
            #rospy.loginfo("Too close L("+str(i)+"):"+str(lasers.ranges[i])+" "+str(cos((10-i)/180.*pi))+" "+str(lasers.ranges[i]*cos((10-i)/180.*pi)))
            wallTooCloseL = True
          elif lasers.ranges[i]*cos((10-i)/180.*pi) < th_wallTooFar:
            wallOKL = True
          elif lasers.ranges[i]*cos((10-i)/180.*pi) < th_neglectedWall:
            wallTooFarL = True

        for i in range(angleRMin,angleRMax):
          if lasers.ranges[i] < distWallRMin:
            distWallRMin = lasers.ranges[i]
          if lasers.ranges[i]*cos((189-i)/180.*pi) < th_wallTooClose:
            #rospy.loginfo("Too close L("+str(i)+"):"+str(lasers.ranges[i])+" "+str(cos((10-i)/180.*pi))+" "+str(lasers.ranges[i]*cos((10-i)/180.*pi)))
            wallTooCloseR = True
          elif lasers.ranges[i]*cos((189-i)/180.*pi) < th_wallTooFar:
            wallOKR = True
          elif lasers.ranges[i]*cos((189-i)/180.*pi) < th_neglectedWall:
            wallTooFarR = True

        #rospy.loginfo("Wall min distances, L:"+str(distWallLMin)+" R:"+str(distWallRMin))
        #rospy.loginfo("Walls L, too close:"+str(wallTooCloseL)+" too far:"+str(wallTooFarL)+" R, too close:"+str(wallTooCloseR)+" too far:"+str(wallTooFarR))

        # Choose policy based on front obstacle and lateral walls detection:
        if obstacleFront:
          if lastWallOnLeft:
            v.speed_left= -v_turn
            v.speed_right= v_turn
          else:
            v.speed_left=   v_turn
            v.speed_right= -v_turn
          rospy.loginfo("Wall Follower: OBSTACLE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooCloseL and (not(wallTooCloseR) or distWallLMin<distWallRMin):
          lastWallOnLeft = True
          v.speed_left=  v_turn
          v.speed_right= v_fwd
          #v.speed_left=  v_fwd
          #v.speed_right= v_turn
          rospy.loginfo("Wall Follower: L TOO CLOSE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooCloseR and (not(wallTooCloseL) or distWallLMin>distWallRMin):
          lastWallOnLeft = False
          v.speed_left=  v_fwd
          v.speed_right= v_turn
          #v.speed_left=  v_turn
          #v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: R TOO CLOSE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallOKL :
          lastWallOnLeft = True
          v.speed_left=  v_fwd
          v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: L OK - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallOKR :
          lastWallOnLeft = False
          v.speed_left=  v_fwd
          v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: R OK - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooFarL and (not(wallTooFarR) or distWallLMin<distWallRMin):
          lastWallOnLeft = True
          v.speed_left=  v_fwd
          v.speed_right= v_turn
          #v.speed_left=  v_turn
          #v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: L TOO FAR - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooFarR and (not(wallTooFarL) or distWallLMin>distWallRMin):
          lastWallOnLeft = False
          v.speed_left=  v_turn
          v.speed_right= v_fwd
          #v.speed_left=  v_fwd
          #v.speed_right= v_turn
          rospy.loginfo("Wall Follower: R TOO FAR - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif lastWallOnLeft:
          v.speed_left=  v_fwd
          v.speed_right= v_turnf
          rospy.loginfo("Wall Follower: LOST WALL, L - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        else:
          v.speed_left=  v_turnf
          v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: LOST WALL, R - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))

        '''
        # OLD CODE, BEFORE FASTSIM UPDATE
        # Choose policy based on front obstacle and lateral walls detection:
        if obstacleFront:
          if lastWallOnLeft:
            v.speed_left= -v_turn
            v.speed_right= v_turn
          else:
            v.speed_left=   v_turn
            v.speed_right= -v_turn
          rospy.loginfo("Wall Follower: OBSTACLE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooCloseL and (not(wallTooCloseR) or distWallLMin<distWallRMin):
          lastWallOnLeft = True
          v.speed_left=  v_turn
          v.speed_right= v_fwd
          #v.speed_left=  v_fwd
          #v.speed_right= v_turn
          rospy.loginfo("Wall Follower: L TOO CLOSE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooCloseR and (not(wallTooCloseL) or distWallLMin>distWallRMin):
          lastWallOnLeft = False
          v.speed_left=  v_fwd
          v.speed_right= v_turn
          #v.speed_left=  v_turn
          #v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: R TOO CLOSE - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallOKL :
          lastWallOnLeft = True
          v.speed_left=  v_fwd
          v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: L OK - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallOKR :
          lastWallOnLeft = False
          v.speed_left=  v_fwd
          v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: R OK - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooFarL and (not(wallTooFarR) or distWallLMin<distWallRMin):
          lastWallOnLeft = True
          v.speed_left=  v_fwd
          v.speed_right= v_turn
          #v.speed_left=  v_turn
          #v.speed_right= v_fwd
          rospy.loginfo("Wall Follower: L TOO FAR - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif wallTooFarR and (not(wallTooFarL) or distWallLMin>distWallRMin):
          lastWallOnLeft = False
          v.speed_left=  v_turn
          v.speed_right= v_fwd
          #v.speed_left=  v_fwd
          #v.speed_right= v_turn
          rospy.loginfo("Wall Follower: R TOO FAR - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        elif lastWallOnLeft:
          v.speed_left=  2
          v.speed_right= 1
          rospy.loginfo("Wall Follower: LOST WALL, L - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        else:
          v.speed_left=  1
          v.speed_right= 2
          rospy.loginfo("Wall Follower: LOST WALL, R - Speed L:"+str(v.speed_left)+" R:"+str(v.speed_right))
        '''

        # publish the suggested movement
        pub.publish(v)
        r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        wallFollower()
    except rospy.ROSInterruptException: pass
