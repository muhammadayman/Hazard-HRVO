from math import ceil, floor, sqrt
import copy
from math import cos, sin, tan, atan2, asin
from math import pi as PI
import numpy


def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2) + 0.001


def RVO_update(X, V_des, V_current, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius'] + 0.01       # kan 0.1
    V_opt = list(V_current)
    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1]]                     # = V_current[i]
        pA = [X[i][0], X[i][1]]                                     # = X[i]
        RVO_BA_all = []
        for j in range(len(X)):
            if i != j:
                vB = [V_current[j][0], V_current[j][1]]             # = V_current[j]
                pB = [X[j][0], X[j][1]]                             # = X[j]
                # use RVO
                # transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]   # RVO apex
                dist_BA = distance(pA, pB)                              # get distance between the 2 agents
                theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])          # get angle of the distance line at pA

                if dist_BA < 2 * ROB_RAD:
                    dist_BA = 2 * ROB_RAD                               # Minimum -> two agents

                theta_BAort = asin(2 * ROB_RAD / dist_BA)
                theta_ort_left = theta_BA + theta_BAort
                theta_ort_right = theta_BA - theta_BAort
                # Collision Cone
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                # use HRVO
                dist_dif = distance([0.5 * (vB[0] - vA[0]), 0.5 * (vB[1] - vA[1])], [0, 0])
                transl_vB_vA = [pA[0] + vB[0] + cos(theta_ort_left) * dist_dif, pA[1] + vB[1] + sin(theta_ort_left) * dist_dif]
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * ROB_RAD]
                RVO_BA_all.append(RVO_BA)

        for hole in ws_model['circular_obstacles'] + ws_model['hazard']:

            vB = [0, 0]
            pB = hole[0:2]
            R = hole[2]                                            # slice notation [0,1]
            transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
            theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
            dist_BA = distance(pA, pB)
            rad = hole[2]
            if (rad + ROB_RAD) > dist_BA:
                dist_BA = rad + ROB_RAD
            if (rad + ROB_RAD) > dist_BA:
                dist_BA = rad + ROB_RAD
            theta_BAort = asin((rad + ROB_RAD) / dist_BA)
            theta_ort_left = theta_BA + theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA - theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def hazard_update(X, V_current, V_des, V_max, ws_model):

    # V_current here is V_opt passed from the RVO function in the same t

    #hazard = ws_model['hazard']
    V_opt = list(V_current)
    p_all = panics(X, V_max, ws_model)
    hazard = ws_model['hazard']
    paniced = []
    for i in range(len(X)):
        add = 0
        for j in range(len(hazard)):
            add += p_all[j][i]
        paniced.append(add)

    V_post = []

    for i in range(len(X)):

        if paniced[i] != 0:                                             # 0.28
            add = [V_des[i][0] * 0.07, V_des[i][1] * 0.07]              # f(x) of panic exponential at x = R
            for j in range(len(hazard)):

                dif_x = [X[i][k] - hazard[j][k] for k in range(2)]
                norm = distance(dif_x, [0, 0])  # hypotenuse
                new_v = [dif_x[k] / norm * p_all[j][i] for k in range(2)]

                add[0] = add[0] + new_v[0]
                add[1] = add[1] + new_v[1]

            # to normalize the magnitude to v_max again
            theta = atan2(add[1], add[0])
            v = [cos(theta) * V_max[i], sin(theta) * V_max[i]]

            V_post.append(v)

        if paniced[i] == 0:
            V_post.append(V_opt[i])

    return V_post


def panics(X, V_max, ws_model):

    # could 've change V_max for paniced agents in a new list.
    r = ws_model['robot_radius']
    hazard = ws_model['hazard']
    dists_all = []
    for h in hazard:
        R = h[2]
        dists = []
        for a in X:
            dist = distance(a, h)
            dists.append(dist)
        dists_all.append(dists)

    panics_all = []
    for i in range(len(hazard)):
        panics = []
        pVmax = []
        for d in dists_all[i]:
            if d < r * 2:
                p = 1.0
            elif d < R * 1.5:
                p = numpy.exp(-1.2 * (d / R)**2)
            else:
                p = 0.0
            panics.append(p)
        panics_all.append(panics)
    return panics_all


def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersection test'
    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []                                                   # Radar different V's in diferent Angles
    for theta in numpy.arange(0, 2 * PI, 0.1):                          # [0, 0.1, 0.2, ...2*PI]
        for rad in numpy.arange(0.02, norm_v + 0.02, norm_v / 10.0):     # min rad = 0.02
            new_v = [rad * cos(theta), rad * sin(theta)]                # rad-theta new composite [x, y]
            suit = True
            freebound = True
            dif_b = [new_v[0] + pA[0], new_v[1] + pA[1]]
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                # Starting the new_v from pA (robot A) then drawing a traingle to get the relative angle (Theta)
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])

                if dif_b[1] > 11:
                    freebound = False
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break

            if freebound and suit:
                suitable_V.append(new_v)
            elif freebound and not suit:
                unsuitable_V.append(new_v)

    # ***** Checking to Add the current Velocity also
    new_v = vA[:]
    suit = True
    freebound = True
    dif_b = [new_v[0] + pA[0], new_v[1] + pA[1]]
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])

        if dif_b[1] > 11:
            freebound = False
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break

    if freebound and suit:
        suitable_V.append(new_v)
    elif freebound and not suit:
        unsuitable_V.append(new_v)
    # ***** Checking to Add the current Velocity also

    if suitable_V:
        # print 'Suitable found'
        vA_post = min(suitable_V, key=lambda v: distance(v, vA))

        # ***** making sure (
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
        # ***** making sure )

    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]                                                         # RVO or HRVO apex
                left = RVO_BA[1]                                                        # left & right bounds
                right = RVO_BA[2]                                                       # [x, y] one unit
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0] + pA[0] - p_0[0], unsuit_v[1] + pA[1] - p_0[1]]      # V from pA, tran. back
                theta_dif = atan2(dif[1], dif[0])                                       # its angle
                theta_right = atan2(right[1], right[0])                                 # angle of right cone
                theta_left = atan2(left[1], left[0])                                    # angle of left cone

                # choose inside RVO with after calculating Tc
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                    if rad <= abs(dist * sin(small_theta)):                            # to calculate on-touch
                        rad = abs(dist * sin(small_theta))
                    big_theta = asin(abs(dist * sin(small_theta)) / rad)
                    dist_tc = abs(dist * cos(small_theta)) - abs(rad * cos(big_theta))
                    if dist_tc < 0:
                        dist_tc = 0
                    tc_v = dist_tc / distance(dif, [0, 0])
                    tc.append(tc_v)
                # approximate

            tc_V[tuple(unsuit_v)] = min(tc) + 0.001
        # Aggressiveness & time to collison
        WT = 0.2
        vA_post = min(unsuitable_V, key=lambda v: ((WT / tc_V[tuple(v)]) + distance(v, vA)))

    return vA_post


def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False


def compute_V_des(X, goal, V_max):
    V_des = []
    rch = []
    for i in range(len(X)):
        # Cos(theta) is already the ratio dif_x / norm. to escape the goal the subtraction is reversed.
        dif_x = [goal[i][k] - X[i][k] for k in range(2)]  # New Velocity to reach goal in one-step
        norm = distance(dif_x, [0, 0])    # watar
        norm_dif_x = [dif_x[k] / norm * V_max[i] for k in range(2)]
        # to get there in one step V_max should = hypotenuse or distance or norm
        '''
        theta = atan2(goal[i][1] - X[i][1], goal[i][0] - X[i][0])
        norm_dif_x = [cos(theta) * V_max[i], sin(theta) * V_max[i]]
        '''
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i]):
            V_des[i][0] = 0
            V_des[i][1] = 0
            rch.append(True)
        else:
            rch.append(False)

    return V_des, rch


def plan_paths(X, goal, exits):
    count_1 = 0
    count_2 = 0
    for i in range(len(goal)):
        if goal[i] == exits[0]:
            count_1 += 1

        else:
            count_2 += 1

    dense_exit = None
    lite_exit = None
    if count_1 > count_2:
        dense_exit = exits[0]
        lite_exit = exits[1]

    else:
        dense_exit = exits[1]
        lite_exit = exits[0]

    heading_to_dense = [i for i, e in enumerate(goal) if e == dense_exit]
    dist_from_dense = [distance(X[i], dense_exit) for i in heading_to_dense]
    # sorting dist and sync index with it
    dist_from_dense, heading_to_dense = (list(t) for t in zip(*sorted(zip(dist_from_dense, heading_to_dense), reverse=True)))
    for i in range(abs(count_1 - count_2) // 2):
        goal[heading_to_dense[i]] = lite_exit


def reach(p1, p2, bound=0.2):
    if distance(p1, p2) < bound:
        return True
    else:
        return False
