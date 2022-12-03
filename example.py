import sys
import os
from vis import visualize_traj_dynamic
from HRVO import *
import random
import numpy as np


def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print('Error: Creating directory. ' + directory)


still = [6]
for na in still:
    agents = '%d_agents' % na
    createFolder(agents)
    for ru in range(25):
        random.seed(ru)
        createFolder(agents + "/" + 'data%d' % ru)
        # ------------------------------
        # define workspace model
        ws_model = dict()
        # robot radius
        ws_model['robot_radius'] = 0.1
        #ws_model['hazard_radius'] = 0.8
        left = [[i * 0.4, 6.5, 0.2] for i in np.arange(0.6, 8.0, 1.0)]
        right = [[i * 0.4, 6.5, 0.2] for i in np.arange(9.9, 17, 1.0)]
        mid_l = [[3.25, 6.45, 0.05], [3.25, 6.35, 0.05]]
        mid_r = [[3.75, 6.45, 0.05], [3.75, 6.35, 0.05]]
        ws_model['circular_obstacles'] = left + mid_l + mid_r + right
        ws_model['hazard'] = [[3.6, 3.5, 1]]
        # ws_model['hazard']= [[2, 3, ws_model['hazard_radius']]]
        ws_model['boundary'] = []

        # ------------------------------.5
        # initialization for robot
        # position of [x,y]
        # X = [[-0.5 + 1.0 * i, 0.0] for i in range(7)] + [[-0.5 + 1.0 * i, 5.0] for i in range(7)]

        step = ws_model['robot_radius'] * 2
        xys = [[x, y] for x in np.arange(3.2, 3.8, step) for y in np.arange(3, 3.5, step)]
        X = random.sample(xys, na)
        X_idx = [i for i in range(len(X))]
        # velocity of [vx,vy
        V = [[0, 0] for i in range(len(X))]
        # maximal velocity norm
        V_max = [1.0 for i in range(len(X))]
        exit_1 = [3.5, 6.6]
        exit_2 = [5.3, 6.6]
        exits = [exit_1]
        goal = [exit_1 for i in range(len(X))]

        #------------------------------
        # simulation setup
        # total simulation time (s)
        total_time = 80
        # simulation step
        step = 0.01
        #------------------------------

        '''
        dif_1 = [exits[0][k] - X[i][k] for i in range(len(X)) for k in range(2)]
        dif_2 = [exits[1][k] - X[i][k] for i in range(len(X)) for k in range(2)]
        '''
        '''
        dis1 = []
        dis2 = []
        for i in range(len(X)):
            norm1 = distance(exits[0], X[i])
            norm2 = distance(exits[1], X[i])
            dis1.append(norm1)
            dis2.append(norm2)

        for i in range(len(X)):
            if dis1[i] < dis2[i]:
                goal[i] = exits[0]
            else:
                goal[i] = exits[1]
        '''
        # simulation starts

        evacuated = []
        evacuated_at_t = []
        t = 0
        while t * step < total_time:

            if not X_idx:
                with open("stats.txt", "a") as f:
                    f.write(str(t / 10) + '\t')       # t/100 to get in Secs
                print("run ", ru, " at agents ", na)
                print("time = ", t / 10)
                break

            for i in range(len(ws_model['hazard'])):
                if t < 180:
                    ws_model['hazard'][i][2] += 0.0005           # hend 0.001 sara 0.0005
                elif t > 180:
                    ws_model['hazard'][i][2] -= 0.0005
                    if ws_model['hazard'][i][2] <= 0.0004:
                        del hazard[i]

            '''
            if t == 20:
                plan_paths(X, goal, exits)
                print("paths optimized at ", t/10)
            '''
            V_des, rch = compute_V_des(X, goal, V_max)

            V = RVO_update(X, V_des, V, ws_model)

            V = hazard_update(X, V, V_des, V_max, ws_model)

        #    just_Stopped = [i for i, e in enumerate(V_des) if e[0] == 0 and e[1] == 0]
        #    just_evacuated = [a for a in just_Stopped if rch[a] == True and a not in evacuated]

            just_evacuated = []
            for i in range(len(X)):
                if V_des[i][0] == 0 and V_des[i][1] == 0 and rch[i] == True:
                    if X_idx[i] not in evacuated:
                        just_evacuated.append(X_idx[i])

            for i in just_evacuated:          # i - idx , j - for loop
                evacuated.append(i)
                evacuated_at_t.append([i, t])
                j = X_idx.index(i)
                X_idx.remove(i)
                del X[j]
                del goal[j]
                del V_des[j]
                del V[j]

            for i in range(len(X)):
                X[i][0] += V[i][0] * step
                X[i][1] += V[i][1] * step

            #----------------------------------------
            # visualization

            if t % 10 == 0:
                # visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))

                visualize_traj_dynamic(ws_model, X, V, goal, X_idx, time=t * step, name=agents + "/" + 'data%d/snap%s.png' % (ru, str(t / 10)))

            just_evacuated.clear()

            #print("time: ", t/10)

            t += 1
    with open("stats.txt", "a") as f:
        f.write('\n')

    '''
    print(X_idx)
    for i in range(len(X_idx)):
        print(goal[X_idx[i]])
    '''

'''
import glob
import cv2

img_array = []
for filename in glob.glob('S:/Academic/1- GP/RMAS/data/*.png'):
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width, height)
    img_array.append(img)


out = cv2.VideoWriter('project.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
'''
