#!/usr/bin/env python

import rospy
import math
import random #used for the random choice of a strategy
from std_msgs.msg import Int16,Float32,Bool,Float32MultiArray,Int16MultiArray
from nav_msgs.msg import Odometry
from fastsim.srv import *
from subsomption.msg import Channel
from sensor_msgs.msg import LaserScan
import sys
import numpy as np
import time
from collections import defaultdict, OrderedDict
from itertools import izip_longest
import csv

channel=[]
lasers=LaserScan()
radar = []
odom = Odometry()
bumper_l=0
bumper_r=0
goalx = 300
goaly = 480

#-------------------------------------------
class CallBack_module_cl(object):

    def __init__(self, num):
        print "Creating callback for "+str(num)
        self.num = num

    def __call__(self, data):
        return callback_module(self.num,data)


#-------------------------------------------
def callback_module(n, data):
    channel[n]=data
    #rospy.loginfo(rospy.get_caller_id()+" n=%d Activated: %d speed_l: %f speed_r: %f",n,data.activated, data.speed_left, data.speed_right)

#-------------------------------------------
def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

#-------------------------------------------
def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)

#-------------------------------------------
def callback_lasers(data):
  global lasers
  lasers=data

#-------------------------------------------
def callback_radar(data):
  global radar
  radar=data.data

#-------------------------------------------
def callback_odom(data):
  global odom
  odom=data

# ajouter un call back radar avec le bon type de donnees

#-------------------------------------------
# nbCh is the number of behavioral modules (channels) in competition
# gatingType sets the gating algorithm to be used ['random','qlearning']
#-------------------------------------------

def strategy_gating(nbCh,gatingType):
    rospy.init_node('strategy_gating', anonymous=True)
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0

    # Parameters of State building
    th_neglectedWall = 35

    angleLMin = 0
    angleLMax = 55

    angleFMin=56
    angleFMax=143

    angleRMin=144
    angleRMax=199

    # The node publishes movement orders for simu_fastsim :
    pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
    pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)

    # The node receives order suggestions by the behavioral modules (channels):
    for n in range(nbCh):
        rospy.Subscriber("/navigation_strategies/channel"+str(n), Channel, CallBack_module_cl(n))

    # If necessary, the node receives sensory information from simu_fastsim:
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
    rospy.Subscriber("/simu_fastsim/radars", Int16MultiArray, callback_radar)
    rospy.Subscriber("/simu_fastsim/odom", Odometry, callback_odom)
    rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)

    # *Targeted* operating frequency of the node:
    frequency = 10 # 10hz
    r = rospy.Rate(frequency) 

    # Q-learning related stuff
    # Algorithm parameters
    alpha = 0.4
    beta = 8
    gamma = 0.9

    # definition of states at time t and t-1
    S_t = ''
    S_tm1 = ''
    Q = defaultdict(int)

    # start time and timing related things
    startT = rospy.get_time()
    rospy.loginfo("Start time: "+str(startT))
    
    trial = 0
    nbTrials = 100
    trialDuration = np.zeros((nbTrials))

    choice = -1
    rew = 0
    
    ts = 0
    bumps = 0
    bumps_list = np.zeros(nbTrials, dtype=int)-1  # number of bumps into a wall for each trial
    
    i2strat = ['wall follower','guidance']

    def draw_proba(Q, S, beta=beta):
      # draw an action according according
      # to the softmax policy
      r = np.random.random()

      Z = sum(np.exp(beta*Q[(S, a)]) for a in range(nbCh))

      cum_probas = np.zeros(nbCh+1)
      cum_probas[1:] = np.array([np.exp(beta*Q[(S, a)])/Z for a in range(nbCh)]).cumsum()

      for i in range(nbCh+1):
          if cum_probas[i] < r <= cum_probas[i+1]:
              return i

    # Main loop:
    while (not rospy.is_shutdown()) and (trial <nbTrials):
      speed_l=0
      speed_r=0
      # processing of the sensory data :
      #------------------------------------------------
      # 1) has the robot found the reward ?
      #rospy.loginfo("pose: "+str(odom.pose.pose.position.x)+", "+str(odom.pose.pose.position.y))
      dist2goal = math.sqrt((odom.pose.pose.position.x-goalx)**2+(odom.pose.pose.position.y-goaly)**2)
      #rospy.loginfo(dist2goal)
      # if so, teleport it:
      if (dist2goal<30):
        rospy.wait_for_service('simu_fastsim/teleport')
        try:
          # teleport robot
          teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
          x  = 300 #20+random.randrange(520)
          y  = 40
          th = random.randrange(360)/2*math.pi
          resp1 = teleport(x, y, th)
          # store information about the duration of the finishing trial:
          currT = rospy.get_time()
          trialDuration[trial] = currT - startT
          bumps_list[trial] = bumps
          startT = currT
          rospy.loginfo("Trial "+str(trial)+" duration:"+str(trialDuration[trial])\
          +" / Nb of bumps into wall: "+str(bumps))
          trial +=1
          bumps = 0
          rew = 1
          odom.pose.pose.position.x = x
          odom.pose.pose.position.y = y
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e

      # 2) has the robot bumped into a wall ?
      #rospy.loginfo("BUMPERS "+str(bumper_r)+' '+str(bumper_l))
      if bumper_r or bumper_l:
        rew = -1
        bumps += 1
        #rospy.loginfo("BING! A wall...")

      # 3) build the state, that will be used by learning, from the sensory data
      #rospy.loginfo("Nb laser scans="+str(len(lasers.ranges)))
      if len(lasers.ranges) == 200:
        S_tm1 = S_t
        S_t   = ''
        # determine if obstacle on the left:
        wall='0'
        for l in lasers.ranges[angleLMin:angleLMax]:
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall
        # determine if obstacle in front:
        wall='0'
        for l in lasers.ranges[angleFMin:angleFMax]:
          #rospy.loginfo("front:"+str(l))
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall
        # determine if obstacle in front:
        wall='0'
        for l in lasers.ranges[angleRMin:angleRMax]:
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall

        # check if we are receiving radar measurements
        if radar != 0:
          radar_list = []
          for i in range(len(radar)):
            radar_list.append(radar[i])
          #rospy.loginfo(str(radar_list))

        S_t += str(radar_list[0])

        #rospy.loginfo("S(t)="+S_t+" ; S(t-1)="+S_tm1)
        
      ts += 1
      # The chosen gating strategy is to be coded here:
      #------------------------------------------------
      if gatingType=='random':
        choice = random.randrange(nbCh)
        #choice = 1
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='randomPersist':
        # a choice is made every 2 seconds
        totalNbSteps = 2*frequency
        if ts % totalNbSteps == 0:
          choice = random.randrange(nbCh)
        
        rospy.loginfo("randomPersist (trial "+str(trial)+"): "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
    
      #------------------------------------------------
      elif gatingType=='guidance':
        choice = 1
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='wallFollower':
        choice = 0
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='qlearning':
        # maximum number of steps between two action choices
        totalNbSteps = 2*frequency

        if ts % totalNbSteps == 0 or S_t != S_tm1:
          Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
          if rew != 0:
            rew = 0
          choice = draw_proba(Q, S_t)
          rospy.loginfo("Q-learning (time: "+str(int(rospy.get_time()-startT))+"): trial "+str(trial)+" / "+i2strat[choice])
          speed_l=channel[choice].speed_left
          speed_r=channel[choice].speed_right
        elif rew != 0:
          Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
          rew = 0
          
        
      #------------------------------------------------
      else:
        rospy.loginfo(gatingType+' unknown.')
        exit()

      #for i in range(nbCh): 
      #  channel[i]=v

      pub_l.publish(speed_l)
      pub_r.publish(speed_r)
      r.sleep()   
    
    # Log files opening
    logDuration = open('DureesEssais_'+gatingType+'_a_'+str(alpha)+'_b_'+str(beta)+'_g_'+str(gamma)+'_'+str(startT),'w')

    def truncate(f):
      return float(format(f, '.2f').rstrip('0').rstrip('.'))
    truncate = np.vectorize(truncate)

    trialDuration = truncate(trialDuration)

    for i in range(nbTrials):
      rospy.loginfo('T = '+str(trialDuration[i])+\
      ' / Nb bumps into wall: '+str(bumps_list[i]))
      logDuration.write(str(i)+' '+str(trialDuration[i])+'\n')
    
    logDuration.close()

    def med_quartiles(durations):
      med = truncate(np.percentile(durations, 50))
      fst_quartile, thrd_quartile = truncate(np.percentile(durations, 25)), truncate(np.percentile(durations, 75))
      return med, fst_quartile, thrd_quartile
    
    med, fst_quartile, thrd_quartile = med_quartiles(trialDuration)

    rospy.loginfo('Median: '+str(med)+'\n')
    rospy.loginfo('1st Quartile: '+str(fst_quartile)+'\n')
    rospy.loginfo('3rd Quartile: '+str(thrd_quartile)+'\n')

    # Saving (Trial, Duration, Nb of bumps, Median, Quartiles) in a log file
    data = [['Trial', 'Duration', 'Number of bumps into walls']]

    data.extend([[x for x in L if x is not None] for L\
     in izip_longest(range(1, nbTrials+1), trialDuration, bumps_list)])
    
    with open('/home/viki/catkin_ws/src/navigation_strategies/'+(str(int(startT))[3:])+'_Trials_'+\
      gatingType+'_a_'+str(alpha)+'_b_'+str(beta)+'_g_'+str(gamma)+'.csv','w') as f:
      csv.writer(f).writerows(data)

    # Saving Median and Quartiles in a log file
    data_stat = [['Trials','Median', '1st Quartile', '3rd Quartile'],\
    ['All', med, fst_quartile, thrd_quartile]]

    if nbTrials > 10:
      med, fst_quartile, thrd_quartile = med_quartiles(trialDuration[:10])
      data_stat.append(['1 to 10', med, fst_quartile, thrd_quartile])

      med, fst_quartile, thrd_quartile = med_quartiles(trialDuration[-10:])
      data_stat.append([str(nbTrials-9)+' to '+str(nbTrials), med, fst_quartile, thrd_quartile])
    
    with open('/home/viki/catkin_ws/src/navigation_strategies/'+(str(int(startT))[3:])+'_Stats_'+\
      gatingType+'_a_'+str(alpha)+'_b_'+str(beta)+'_g_'+str(gamma)+'.csv','w') as f:
      csv.writer(f).writerows(data_stat)

    if gatingType=='qlearning':
      # Storing the Q-values at the end
      keys = list(OrderedDict.fromkeys(['1110', '1117', '0000', '0007']+[state for state, _ in Q.keys() if state]))
      rospy.loginfo('States: '+repr(keys)+'\n')


      with open('/home/viki/catkin_ws/src/navigation_strategies/'+(str(int(startT))[3:])+'_Q-values_'+\
        'nbTrials_'+str(nbTrials)+'_a_'+str(alpha)+'_b_'+str(beta)+'_g_'+str(gamma)+'.csv','w') as f:
        csv.writer(f).writerow(["State", "Wall follower", "Guidance"])
        for key in keys:
            csv.writer(f).writerow([key]+ [Q[(key, i)] for i in range(nbCh)])
  

#-------------------------------------------
if __name__ == '__main__':

    nbch=int(sys.argv[1])
    gatingType=sys.argv[2]
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0
    channel=[v for i in range(nbch)]
    try:
        strategy_gating(nbch,gatingType)
    except rospy.ROSInterruptException: pass
