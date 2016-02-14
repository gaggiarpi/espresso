import numpy as np

def filter_initialize():
    global Q0, R, x, P
    Q0 =np.array([[0.08, 0.00], 
                  [0.00, 0.08]])
    R = np.array([[2.0, -1.0],
                  [-1.0, 2.0]])
    x = np.array([0.0, 0.0])
    P = np.array([[0.0, 0.0],
                  [0.0, 0.0]])

def filter(current_weight, prev_weight, time, prev_time, seconds_elapsed, Q0, R):
    global x, P, filtered_weight, filtered_flow
    dt = time - prev_time
    z = [current_weight, (current_weight-prev_weight)/dt] 
    F = np.array([[1.0, dt],
                  [0.0, 1.0]])
    if seconds_elapsed < 7: 
        Q = Q0 * (seconds_elapsed/7.0)**4
        # Filter super aggressively early: weight should be zero anyway.
    else:
        Q = Q0
    x = np.dot(F, x)
    P = np.dot(np.dot(F, P), F.T) + Q
    y = z - x
    if shot_pulling == False: # At the end of shot: progressively reduce the amount of filtering to 0.
        seconds_after_end = time.time() - end_shot_time
        if  seconds_after_end < 1.5:
            R = R * (1.5 - seconds_after_end)/1.5
        else:
            R = R * 0
    K = np.dot(P, np.linalg.inv(P + R))
    x = x + np.dot(K, y)
    P = P - np.dot((np.eye(2) - K), P)
    filtered_weight = x[0]
    filtered_flow = x[1]


########
# Test #
########

import json
import time

d = json.load(open("/home/pi/logs/log_shot2016-02-04-1408.json"))

weights = d["full_weight_series"]
times = d["full_weight_series_time"]
start = d["start"]

filter_initialize()
for i in range(1, len(weights)):
    if times[i-1]-start < 0:
        next
    else:
        t1 = times[i]-start
        t0 = times[i-1] - start
        t = time.time()
        filter(weights[i], weights[i-1], t1, t0, t1, Q0, R)
        wt = time.time() - t
        print "t = %.02f, w = %0.2f, f = %0.2f, computed in %s" %(t1, filtered_weight, filtered_flow, wt)
        time.sleep(times[i+1]-start-t1)


    
# taring and preheat:
#    filter_initialize()    
# if shot_pouring & already_tared:
#    filter()
    
### in pour_shot() ###
# replace: current_weight by filtered_weight
# replace: flow_per_second by filtered_flow

### in end_shot()
# replace: 