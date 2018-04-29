---
title: "Tutorial 2: Navigation Strategies"
author:
- 'Younesse Kaddar'
- 'Kexin Ren'
date: 2018-04-28
tags:
  - lab
  - tutorial
  - exercise
  - reinforcement-learning
  - naigation-strategies
  - ROS
  - robotics
  - neuroscience
  - neuro-robotique
  - neurorobotics
  - Girard
  - Coninx
abstract: 'Lab 2: Navigation Strategies'
---

# Tutorial 2: Navigation Strategies

### Kexin Ren & Younesse Kaddar (**Lecturers**: Alexandre Coninx & Benoît Girard)


Summary of the problem (do we need this part?)

Questions


## 1. Notice that the `random` policy is very unlikely to get the robot out the dead-end region of the map. In a first phase, write a policy called `randomPersist`, where one the two strategies is uniformly drawn at random is left unchanged for 2 seconds (instead of changing at each time step). This increased stability should enable the robot to have an actual chance of getting out of the dead-end.

Since a choice is made every $2$ seconds, we first define a `ts` variable as the current number of steps:
`ts = 0`

Besides, we also define the total number of steps within $2$ seconds as `totalNbSteps`:

```python
totalNbSteps = 2*frequency
```

Then, the `randomPersist` function is defined as follows:

```python
elif gatingType=='randomPersist':
# a choice is made every 2 seconds
totalNbSteps = 2*frequency
if ts % totalNbSteps == 0:
  choice = random.randrange(nbCh)

rospy.loginfo("randomPersist (trial "+str(trial)+"): "+i2strat[choice])
speed_l=channel[choice].speed_left
speed_r=channel[choice].speed_right
```

## Execute 10 trials with this new strategy and save each trial duration (to reach the goal). Compute the median, the first and the third quartiles.

We use the following code to calculate and print the median, 1st quartile and 3rd quartile of the data:

```python
def med_quartiles(durations):
      med = truncate(np.percentile(durations, 50))
      fst_quartile, thrd_quartile = truncate(np.percentile(durations, 25)), truncate(np.percentile(durations, 75))
      return med, fst_quartile, thrd_quartile

    med, fst_quartile, thrd_quartile = med_quartiles(trialDuration)

    rospy.loginfo('Median: '+str(med)+'\n')
    rospy.loginfo('1st Quartile: '+str(fst_quartile)+'\n')
    rospy.loginfo('3rd Quartile: '+str(thrd_quartile)+'\n')
```

After running 10 trials with randomPersist strategy, we obtained the following statistical results:

```
Median of the trial duration:
1st quartile of the trial duration:
3rd quartile of the trial duration:
```

## 3. You're about to implement a `qlearning` policy similar to the one used by Dollé et al. (2010): it uses a Q-learning algorithm to learn, trial after trial, which is the best strategy depending on the state of the surrounding world.

- **Definition of the states**: in our case, states are defined as a 4-digits string. The first three ones indicate whether a wall has been detected at less than 35 distance units: respectively on the left, in front, and on the right of the robot. The 4th digit range from $0$ to $7$ and indicates in which region the radar has detected the goal ($0$ corresponds the front-left region, and it goes counter-clockwise). The current (resp. previous) state is available in the `S_t` (resp. `S_tm1`) variable.

- **Q-learning**: the bottom line of Q-learning is to maintain a table of values $Q(s, a)$ for each state $s$ and action $a$. Here, the states are defined as above, and the actions $a ∈ \lbrace 0, 1 \rbrace$ correspond to the two strategies `wallFollower` and `guidance`.

    At each time step $t$, the prediction error is:

    $$δ(t) = rew(t) + γ \max_{a_i} Q(s(t), a_i) - Q(s(t-dt), a(t-dt))$$

    Q-values are updated as follows:

    $$Q(s(t-dt), a(t-dt)) ← Q(s(t-dt), a(t-dt)) + α δ(t)$$

    Finally, actions are chosen according to the softmax policy:

    $$p(a_i \mid s(t)) = \frac{\exp\big(β Q(s(t), a_i)\big)}{\sum\limits_{ j } \exp(β Q(s(t), a_j))}$$

In a first time, it is suggested that you use the parameters $α = 0.4, β = 8, γ = 0.9$. As it happens, time varies continuously here, and strategies processing the information send new motor commands at a $10 \text{ Hz}$ frequency. It is unlikely that the sensory state changes notably at each time step. Likewise, just because you're selection an action doesn't mean that the state will change at the next step. When it comes to learning, we will set the working time step to be $2$ seconds.

You are asked to run the algorithm on a hybrid synchronous-asynchronous mode, that is:

- a new action is made

    - either when $2$ seconds have elapsed since the last choice
    - or as soon as the sensory state changes

- Q-values are updated

    - either when an action has just been chosen
    - or when the robot has just received a non-zero reward (bump into a wall or goal reached).
Similar as in Q1, we involved totalNbSteps and ts variables in the qlearning function. And according to the Q Learning formulas given in the handout, we define the Q value as:

```python
Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)]);

and we define action choice using softmax policy as:
def draw_proba(Q, S, beta=beta):
  # draw an action according according
  # to the softmax policy
  r = np.random.random()

  Z = sum(np.exp(beta*Q[(S, a)]) for a in range(nbCh))

  cum_probas = np.zeros(nbCh+1)
  cum_probas[1:] = np.array([np.exp(beta*Q[(S, a)])/Z for a in range(nbCh)]).cumsum()
```

Thus, the qlearning strategy is defined as the follows:

```python
elif gatingType=='qlearning':
  # maximum number of steps between two action choices
  totalNbSteps = 2*frequency

  if ts % totalNbSteps == 0 or S_t != S_tm1:
    Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
    rospy.loginfo(str((S_tm1, choice))+" -> "+str(Q[(S_tm1, choice)])+" / rew: "+str(rew))
    if rew != 0:
rew = 0
    choice = draw_proba(Q, S_t)
  elif rew != 0:
    Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
    rospy.loginfo(str((S_tm1, choice))+" -> "+str(Q[(S_tm1, choice)])+" / rew: "+str(rew))
    rew = 0

  rospy.loginfo("Q-learning (time: "+str(int(rospy.get_time()-startT))+"): trial "+str(trial)+" / "+i2strat[choice])
  speed_l=channel[choice].speed_left
  speed_r=channel[choice].speed_right
```
Similar as in Q1, we involved totalNbSteps and ts variables in the qlearning function. And according to the Q Learning formulas given in the handout, we define the Q value as:

```python
Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)]);

and we define action choice using softmax policy as:
def draw_proba(Q, S, beta=beta):
  # draw an action according according
  # to the softmax policy
  r = np.random.random()

  Z = sum(np.exp(beta*Q[(S, a)]) for a in range(nbCh))

  cum_probas = np.zeros(nbCh+1)
  cum_probas[1:] = np.array([np.exp(beta*Q[(S, a)])/Z for a in range(nbCh)]).cumsum()
```

Thus, the qlearning strategy is defined as the follows:

```python
elif gatingType=='qlearning':
  # maximum number of steps between two action choices
  totalNbSteps = 2*frequency

  if ts % totalNbSteps == 0 or S_t != S_tm1:
    Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
    rospy.loginfo(str((S_tm1, choice))+" -> "+str(Q[(S_tm1, choice)])+" / rew: "+str(rew))
    if rew != 0:
rew = 0
    choice = draw_proba(Q, S_t)
  elif rew != 0:
    Q[(S_tm1, choice)] += alpha*(rew+gamma*max(Q[(S_t, a)] for a in range(nbCh))-Q[(S_tm1, choice)])
    rospy.loginfo(str((S_tm1, choice))+" -> "+str(Q[(S_tm1, choice)])+" / rew: "+str(rew))
    rew = 0

  rospy.loginfo("Q-learning (time: "+str(int(rospy.get_time()-startT))+"): trial "+str(trial)+" / "+i2strat[choice])
  speed_l=channel[choice].speed_left
  speed_r=channel[choice].speed_right
```

## 4. Finally, you'll try to quantitatively evaluate how this algorithm works by running it on $30$ trials for instance (the more trials the better).


### Compute the median and quartiles over the first ten trials and the last ten ones: is there an improvement?
We have run 50 trials with the parameters $α = 0.4, β = 8, γ = 0.9$. The statistics of the trial duration over the first 10 trials, last 10 trials and all 50 trials are reported in the following chart:

| Trials  | Median | 1st quartile  | 3rd quartile |
| ------------- | ------------- | ------------- | ------------- |
| 1 to 10  | 69.54  | 54.63  | 101.78  |
| 41 to 50  | 42.22  | 35.38  | 54.46  |
| All  | 47.26  | 34.44  | 67,99  |

The table above indicates an improvment in the last 10 trials comparing with the first 10 trials. Specifically, the median decreased from 69.54 sec (1 to 10 trails) to 42.22 sec (41 to 50 trials), the 1st quartile decreased from 54.63 sec to 35.38 sec, and the 3rd quartile decreased from 101.78 sec to 54.46 sec.

We have implenmented another 50 trials, though there was an increasement in the 1st quartile in the last 10 trials, the statistics shown below still indicated an improvement:

| Trials  | Median | 1st quartile  | 3rd quartile |
| ------------- | ------------- | ------------- | ------------- |
| 1 to 10  | 70.23  | 37.73  | 118.30  |
| 41 to 50  | 64.03  | 46.35  | 70.29  |
| All  | 44.91  | 37.01  | 69.27  |

### Do the number of bumps into a wall decrease?

The number of bumps into the wall decreased in the last 10 trials comparing with the first 10 trials. The average number of bumps in the first 10 trials is 4.2, and in the last 10 trials becomes 0.4. The statistics of the number of bumps are shown as below:

| Trials  | Average | Median | 1st quartile  | 3rd quartile |
| ------------- | ------------- | ------------- | ------------- | ------------- |
| 1 to 10  | 4.2  | 3.5  | 1.25  | 7  |
| 41 to 50  | 0.4  | 0  | 0  | 0  |
| All  | 2.36  | 0  | 0  | 2.75  |

In our another 50 trials, similarly, the average bumps of the last 10 trials is 0.5, much lower than the that of the first 10 trials which is 5.4.

### Even if there doesn't seem to be any improvement (which is likely, with so few trials), store the Q-values at the end and check if the learning goes as expected: look up the Q-values of the `1110`, `1117`, `0000` and `0007` states: what do you observe?

To obatain better accuracy, we implemented 100 trials with the default parameters for four more times. We have observed that:

(1) For states `1110` and `1117`, as expected, when the goal is in front of the robot but is obstructed by the wall, the robot prefers the "wallFollower" strategy to bypass the wall and approach the goal.

(2) However, for the states `0000` and `0007`in which the goal is directly in front of the robot without obstacle, we obtained results different to our expectation -- there is no obvious preference between "wallFollower" and "guidance" strategies and they even show a preference to the "wallFollower“ strategy somewhat which is not effective at all.


We think that the second observation stated above might result from the reward issues: 

(1) When the wall is in front of the robot but the robot has not detected the wall, if the robot uses "Guidance" strategy, it will bump into the wall receive penalty (reward = -1). This negative reward may be reinforced in the trails, and the robot has learned to not use "Guidance" strategy when the wall is not detected even it has already bypassed the wall. 

(2) Moreover, the rebot does not receive any reward by choosing "Guidance" strategy when thre is no wall in front of it, because most of the time, there is still a long way for the robot to go to reach the reward, thus it is not positively reinforced to do so.


To solve this problem, we think we can modify the reward rules in these ways:

(1) A very effective improvement would be to correlated the rewards with distance between the robot and the goal. For example, the closer the robot is to the goal, the bigger the reward. Or if the robot get closer to the goal, it gets a positive reward; if it bumps into the wall, it gets a negative reward; if it is following the wall, it gets neither reward nor penalties. for instance) 

(2) Another less effective way would be to make the reward bigger instead of saying that the trial ends when the robot's distance to the reward is less than 30, we could broaden the radius and specify 50 for instance.


### If you have the time, repeat the experiment with other values of the $α, β$ and $γ$ parameters to see how the learning speed is impacted.



(I inserted the gif here for future use, same order as in Gyazo)
<img src="https://gyazo.com/b3ed317027c72fbd2d2052d22cb25384.gif" alt="1" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/96d5c023e4d6bfe45c14fae02a94f3f6.gif" alt="2" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/656bbbc108adad287280847734f0b098.gif" alt="3" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/53fed15dd462cce1405ce43f00541db3.gif" alt="4" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/1396f632e3799e692d9f5ee07ed4d164.gif" alt="5" style="width: 60%; margin-left: 20%;"/>
