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

## 4. Finally, you'll try to quantitatively evaluate how this algorithm works by running it on $30$ trials for instance (the more trials the better).


### Compute the median and quartiles over the first ten trials and the last ten ones: is there an improvement?


### Do the number of bumps into a wall decrease?

### Even if there doesn't seem to be any improvement (which is likely, with so few trials), store the Q-values at the end and check if the learning goes as expected: look up the Q-values of the `1110`, `1117`, `0000` and `0007` states: what do you observe?

### If you have the time, repeat the experiment with other values of the $α, β$ and $γ$ parameters to see how the learning speed is impacted.


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
(I inserted the gif here for future use)
<img src="https://gyazo.com/b3ed317027c72fbd2d2052d22cb25384.gif" alt="1" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/96d5c023e4d6bfe45c14fae02a94f3f6.gif" alt="2" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/656bbbc108adad287280847734f0b098.gif" alt="3" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/53fed15dd462cce1405ce43f00541db3.gif" alt="4" style="width: 60%; margin-left: 20%;"/>
<img src="https://gyazo.com/1396f632e3799e692d9f5ee07ed4d164.gif" alt="5" style="width: 60%; margin-left: 20%;"/>
