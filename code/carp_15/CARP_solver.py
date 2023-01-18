#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
from random import *
import time
import sys
import getopt
import copy
import multiprocessing
# import plot

# global start_time
# start_time = time.time()

time.clock()
TERMINATION_TIME = 60
RANDOM_SEED = 1.0
trend = {}
# filename = "./CARP_samples/val1A.dat"
# filename = "./CARP_samples/val4A.dat"
# filename = "./CARP_samples/val7A.dat"
# filename = "./CARP_samples/gdb1.dat"
# filename = "./CARP_samples/gdb10.dat"
# filename = "./CARP_samples/egl-e1-A.dat"
filename = "CARP_samples/egl-s1-A.dat"

opts, args = getopt.getopt(sys.argv[2:], "t:s:")
try:
    filename = sys.argv[1]
except:
    pass

try:
    TERMINATION_TIME = float(opts[0][1])
except:
    pass

try:
    RANDOM_SEED = float(opts[1][1])
except:
    pass

# print(filename, TERMINATION_TIME, RANDOM_SEED)
seed(RANDOM_SEED)
TERMINATION_TIME = TERMINATION_TIME - 5
ENABLE_DP = False
def dprint(*a):
    if ENABLE_DP:
        print(a)

file = open(filename)
readed = file.read()
lines = readed.split('\n')
for i in range(8):
    lines[i] = lines[i].split(' : ')

NAME = lines[0][1]
VERTICES = int(lines[1][1])
DEPOT = int(lines[2][1])
REQUIRED_EDGES= int(lines[3][1])
NON_REQUIRED_EDGES = int(lines[4][1])
VEHICLES = int(lines[5][1])
CAPACITY = int(lines[6][1])
TOTAL_COST_OF_REQUIRED_EDGES = int(lines[7][1])

edges = []
costs = []
demands = []
dists = []
prevs = []
sorted_dists = []
for i in range(9,len(lines)-1):
    data = lines[i].split('   ')
    edges.append((int(data[0]), int(data[1])))
    costs.append(int(data[2]))
    demands.append(int(data[4]))

cost_matrix = np.zeros([VERTICES, VERTICES]) + sys.maxint
demand_matrix = np.zeros([VERTICES, VERTICES])
for i in range(VERTICES):
    cost_matrix[i][i] = 0
for i in range(len(edges)):
    e = edges[i]
    cost_matrix[e[0] - 1, e[1] - 1] = costs[i]
    cost_matrix[e[1] - 1, e[0] - 1] = costs[i]
    demand_matrix[e[0] - 1, e[1] - 1] = demands[i]
    demand_matrix[e[1] - 1, e[0] - 1] = demands[i]

def dijkstra(start):
    dist = np.zeros([VERTICES]) + sys.maxint
    dist[start-1] = 0
    flag = []
    prev = []
    for i in range(VERTICES):
        flag.append(0)
        prev.append(-1)
        dist[i] = cost_matrix[start-1][i]
        if dist[i] < sys.maxint:
            prev[i] = start - 1
    flag[start-1] = 1

    k = 0
    for i in range(VERTICES):
        min_cost = sys.maxint
        for j in range(VERTICES):
            if not flag[j] and dist[j] < min_cost:
                min_cost = dist[j]
                k = j
        flag[k] = 1

        for j in range(VERTICES):
            if not flag[j]:
                temp = min_cost + cost_matrix[j, k]
                if temp < dist[j]:
                    dist[j] = temp
                    prev[j] = k

    return dist, prev


for i in range(VERTICES):
    ids = range(VERTICES)
    dist, prev = dijkstra(i + 1)
    dists.append(dist)
    prevs.append(prev)
    d = dict(zip(ids, dist))
    sorted_dist = sorted(d.items(), key=lambda d: d[1])
    sorted_dists.append(sorted_dist)

dprint("dijk time:", time.clock())
# 贪心算法 dijkstra生成每个点之间的最短路径，直接载物直到超载则回去终点或中途回到终点则添加一条路径
def greed():
    demand_matrix_copy = demand_matrix.copy()
    demand_sum = np.sum(demands)

    arcs = []
    while demand_sum > 0:
        arc = []
        carry = 0
        pos = DEPOT
        perm = range(VERTICES)
        while carry < CAPACITY:
            sorted_dist = sorted_dists[pos - 1]
            if carry > 0 and pos == DEPOT:
                break
            for vc in sorted_dist:
                v = vc[0]
                shuffle(perm)
                for i in perm:
                    demand = demand_matrix_copy[v,i]
                    if demand > 0 and carry + demand <= CAPACITY:
                        demand_sum -= demand_matrix[v,i]
                        carry += demand_matrix[v,i]
                        demand_matrix_copy[i,v] = 0
                        demand_matrix_copy[v,i] = 0
                        arc.append((v + 1,i + 1))
                        pos = i + 1
                        break
                else:
                    continue
                break
            else:
                break
        arcs.append(arc)
    return arcs


def calCost(arcs):
    cost_sum = 0
    for arc in arcs:
        cost_sum += calArcCost(arc)
    return cost_sum

def calArcCost(arc):
    cost_sum = 0
    pos = DEPOT
    for e in arc:
        cost_sum += dists[pos - 1][e[0] - 1]
        cost_sum += cost_matrix[e[0] - 1, e[1] - 1]
        pos = e[1]
    cost_sum += dists[pos - 1][DEPOT - 1]
    return cost_sum

def selfing(arcs, cost, startTime, pi, Pt = 0.4, Ps = 0.6, Pc = 0.3):
    seed(RANDOM_SEED + pi)
    g = 0
    last_g = 0
    last_print_time = time.clock()
    lowest_cost = cost

    best_arcs = arcs
    best_cost = lowest_cost
    while time.clock() < TERMINATION_TIME - startTime:
        g += 1
        if time.clock() - last_print_time > 10:
            last_print_time = time.clock()
            dprint(time.clock(), g, last_g, lowest_cost)

        gd = g - last_g

        if gd > 60000:
            dprint('restart')
            if lowest_cost < best_cost:
                best_arcs = arcs
                best_cost = lowest_cost
            arcs = greed()
            lowest_cost = calCost(arcs)
            g = last_g

        child = copy.deepcopy(arcs)
        op = []

        if gd >= 2:
            if random() < Pt:
                transposition(child)
                op.append("tp")
            if random() < Ps:
                shift(child)
                op.append("sh")
            if random() < Pc:
                crossover(child)
                op.append("co")

            if not checkFeasibility(child):
                continue

        m_level = 0
        if gd < 2:
            m_level = 3
        elif gd > 20000:
            m_level = 3
        elif gd > 10000:
            m_level = 2
        elif gd > 500:
            m_level = 1


        mutation(child, m_level)
        op.append(m_level)

        while [] in child:
            child.remove([])
        child.append([])

        child_cost = calCost(child)

        if child_cost < lowest_cost:
            lowest_cost = child_cost
            arcs = child
            last_g = g
            dprint(pi, g, gd, lowest_cost, op)

    dprint(best_cost, lowest_cost)
    if lowest_cost < calCost(best_arcs):
        best_arcs = arcs
        best_cost = lowest_cost
    return best_arcs, best_cost, g

# 遗传算法
def genetic(INIT_M = 30, M = 1, Pt = 0.5, Ps = 0.5, Pc = 0.5):

    # genetic_time = time.time()
    best_arcs = []
    lowest_cost = sys.maxint
    population = []
    costs = []

    if INIT_M < M:
        INIT_M = M

    for i in range(INIT_M):
        arcs = greed()
        cost = calCost(arcs)
        dprint("greed time:", time.clock(), cost)

        if cost < lowest_cost:
            lowest_cost = cost
            best_arcs = arcs
        arcs.append([])
        population.append(arcs)
        costs.append(cost)

    fit_population = []
    fit_costs = []
    for i in range(M):
        fit_cost = sys.maxint
        fit_i = -1
        for j in range(len(costs)):
            cost = costs[j]
            if len(fit_population) > 0 and population[j] == fit_population[len(fit_population)-1]:
                continue
            if cost < fit_cost:
                fit_cost = cost
                fit_i = j

        fit_population.append(population[fit_i])
        fit_costs.append(costs[fit_i])
        costs[fit_i] = sys.maxint

    population = fit_population
    costs = fit_costs

    results = []
    pool = multiprocessing.Pool(processes=M)
    for pi in range(M):
        results.append(pool.apply_async(selfing, (population[pi], costs[pi], time.clock(), pi,)))

    pool.close()
    pool.join()

    g = 0
    total_cost = 0
    for ri in results:
        result = ri.get()
        arcs = result[0]
        cost = result[1]
        g += result[2]
        total_cost += cost
        if cost < lowest_cost:
            lowest_cost = cost
            best_arcs = arcs

    dprint(total_cost, total_cost / M)
    dprint(g, g/M)
    best_arcs.remove([])
    return best_arcs, lowest_cost

def checkFeasibility(child):

    for arc in child:
        carry = 0
        for e in arc:
            carry += demand_matrix[e[0] - 1,e[1] - 1]
        if carry > CAPACITY:
            return False
    return True


def checkIntegrity(child, parent):
    ca = []
    pa = []
    for chrome in child:
        ca += chrome
    for chrome in parent:
        pa += chrome

    ca = getEdgeIndex(ca)
    pa = getEdgeIndex(pa)
    if len(ca) != len(pa):
        print('len')
        sys.exit()
    for ei in pa:
        if ei not in ca:
            e = edges[ei]
            print(e)
            print(child)
            print(parent)
            sys.exit()

def getEdgeIndex(arc):
    edge_indexs = []
    for ei in range(len(arc)):
        e1 = arc[ei]
        for ej in range(len(edges)):
            e2 = edges[ej]
            if (e1[0] == e2[0] and e1[1] == e2[1]) or (e1[0] == e2[1] and e1[1] == e2[0]):
                edge_indexs.append(ej)

    return edge_indexs

def transposition(child):
    if len(child) - 2 <= 0:
        return
    chrome1 = randint(0, len(child) - 2)
    chrome2 = randint(0, len(child) - 2)
    if chrome1 == chrome2:
        return
    start1 = randint(0, max(len(child[chrome1]) - 1 ,0))
    start2 = randint(0, max(len(child[chrome2]) - 1 ,0))
    max_n = min(len(child[chrome1]) - start1, len(child[chrome2]) - start2)
    n = randint(1, max(max_n, 1))

    temp = child[chrome1][start1:(start1 + n)]
    child[chrome1][start1:(start1 + n)] = child[chrome2][start2:(start2 + n)]
    child[chrome2][start2:(start2 + n)] = temp


def shift(child):
    if len(child) - 2 <= 0:
        return
    chrome1 = randint(0, len(child) - 2)
    chrome2 = randint(0, len(child) - 1)
    while chrome1 == chrome2:
        chrome2 = randint(0, len(child) - 1)
    start1 = randint(0, max(len(child[chrome1]) - 1 ,0))
    start2 = randint(0, max(len(child[chrome2]), 0))
    max_n = len(child[chrome1]) - start1
    n = randint(1, max(max_n, 1))

    temp = child[chrome1][start1:(start1 + n)]
    child[chrome1] = child[chrome1][:start1] + child[chrome1][(start1 + n):]
    child[chrome2] = child[chrome2][:start2] + temp + child[chrome2][start2:]


def crossover(child):
    if len(child) - 2 <= 0:
        return
    chrome1 = randint(0, len(child) - 2)
    chrome2 = randint(0, len(child) - 2)
    if chrome1 == chrome2:
        return
    # print len(child[chrome1]) - 1
    start1 = randint(0, max(len(child[chrome1]) - 1, 0))
    start2 = randint(0, max(len(child[chrome2]) - 1, 0))
    # max_n = min(len(child[chrome1]) - start1, len(child[chrome2]) - start2)
    # n = randint(1, max(max_n, 1))

    temp = child[chrome1][start1:]
    child[chrome1] = child[chrome1][:start1] + child[chrome2][start2:]
    child[chrome2] = child[chrome2][:start2] + temp

def mutation(child, m_level):
    # print 'mutation'
    for arci in range(len(child)):
        arc = child[arci]
        new_arc = copy.deepcopy(arc)

        for eii in range(len(arc)):
            e = arc[eii]
            new_arc[eii] = (e[1], e[0])
            if calArcCost(new_arc) > calArcCost(arc):
                new_arc[eii] = (e[0], e[1])
        #
        for ei in range(len(arc)):
            for ej in range(ei + 1, len(arc)):
                if ej - ei == 1 and m_level >= 1:
                    e1 = arc[ei]
                    e2 = arc[ej]
                    new_arc[ei] = (e2[0], e2[1])
                    new_arc[ej] = (e1[0], e1[1])
                    if calArcCost(new_arc) > calArcCost(arc):
                        new_arc[ei] = (e1[0], e1[1])
                        new_arc[ej] = (e2[0], e2[1])
                    else:
                        arc = copy.deepcopy(new_arc)
                elif ej - ei == 2 and m_level >= 2:
                    e1 = arc[ei]
                    em = arc[ei + 1]
                    e2 = arc[ej]
                    new_arc[ei] = (e2[0], e2[1])
                    new_arc[ei + 1] = (e1[0], e1[1])
                    new_arc[ej] = (em[0], em[1])
                    if calArcCost(new_arc) > calArcCost(arc):
                        new_arc[ei] = (e1[0], e1[1])
                        new_arc[ej] = (e2[0], e2[1])
                        new_arc[ei + 1] = (em[0], em[1])
                    else:
                        arc = copy.deepcopy(new_arc)

                elif ej - ei > 2 and m_level >= 3:
                    sub_arc = new_arc[ei + 1:ej]
                    list.reverse(sub_arc)
                    for ek in range(len(sub_arc)):
                        e = sub_arc[ek]
                        sub_arc[ek] = (e[1], e[0])
                    new_arc[ei + 1:ej] = sub_arc
                    if len(new_arc) != len(child[arci]):
                        dprint(ei, ej)
                        dprint(sub_arc, new_arc, child[arci])

                    if calArcCost(new_arc) > calArcCost(arc):
                        list.reverse(sub_arc)
                        for ek in range(len(sub_arc)):
                            e = sub_arc[ek]
                            sub_arc[ek] = (e[1], e[0])
                        new_arc[ei + 1:ej] = sub_arc
                        if len(new_arc) != len(child[arci]):
                            dprint(ei, ej)
                            dprint(sub_arc, new_arc, child[arci])
                    else:
                        arc = copy.deepcopy(new_arc)
                        if len(new_arc) != len(child[arci]):
                            dprint(ei, ej)
                            dprint(sub_arc, new_arc, child[arci])

        child[arci] = new_arc

def output(arcs, cost):
    s = 's '
    for arc in arcs:
        s += '0,'
        for e in arc:
            s += e.__str__() + ','
        s += '0,'

    s = s.replace(', ', ',')
    s = s[:-1]
    print(s)

    q = 'q ' + str(int(cost))
    print(q)

def main():

    cores = multiprocessing.cpu_count() - 1
    best_arcs, best_cost = genetic(M=cores)
    dprint("ga time:", time.clock())

    output(best_arcs, best_cost)


if __name__ == "__main__":
    main()