import sys
import copy
import time
import heapq

import numpy as np

VERTICES_NUM_plus1 = 0
DEPOT = 0
CAPACITY = 0
demand = {}
cost = {}
short_distance = []


def read_info(file):
    input_ = open(file)
    contents = input_.readlines()
    input_.close()
    information = {contents[i].replace('\n', '').split(' : ')[0]: int(contents[i].replace('\n', '').split(' : ')[1]) for
                   i in range(1, 8)}
    global VERTICES_NUM_plus1, DEPOT, CAPACITY, demand, cost, short_distance
    VERTICES_NUM_plus1 = int(information['VERTICES']) + 1
    DEPOT = int(information['DEPOT'])
    CAPACITY = int(information['CAPACITY'])
    short_distance = np.full((VERTICES_NUM_plus1, VERTICES_NUM_plus1), 200000000)
    row, col = np.diag_indices_from(short_distance)
    short_distance[row, col] = 0
    for i in range(9, len(contents) - 1):
        # print(contents[i])
        v1, v2, c, d = map(int, contents[i].replace('\n', '').split())
        # print(v1, v2, c, d)
        cost[(v1, v2)] = c
        cost[(v2, v1)] = c
        short_distance[v1][v2] = c
        short_distance[v2][v1] = c
        if d != 0:
            demand[(v1, v2)] = d
            demand[(v2, v1)] = d
    # print(short_distance)
    # print(cost)
    # print(demand)


def Floyd():
    global short_distance, VERTICES_NUM_plus1
    for k in range(1, VERTICES_NUM_plus1):
        for i in range(1, VERTICES_NUM_plus1):
            for j in range(1, VERTICES_NUM_plus1):
                if short_distance[i][k] + short_distance[k][j] < short_distance[i][j]:
                    short_distance[i][j] = short_distance[i][k] + short_distance[k][j]


def better(u, uj, rule, cur_load):
    if rule == 0:
        return short_distance[u[1]][DEPOT] > short_distance[uj[1]][DEPOT]
    elif rule == 1:
        return short_distance[u[1]][DEPOT] < short_distance[uj[1]][DEPOT]
    elif rule == 2:
        return demand[u] / cost[u] > demand[uj] / cost[uj]
    elif rule == 3:
        return demand[u] / cost[u] < demand[uj] / cost[uj]
    else:
        if cur_load <= 0.5 * CAPACITY:
            return short_distance[u[1]][DEPOT] > short_distance[uj[1]][DEPOT]
        else:
            return short_distance[u[1]][DEPOT] < short_distance[uj[1]][DEPOT]


def path_scanning(rule):
    k = 0
    free = [task for task in demand.keys()]
    routes = []
    loads = []
    costs = []
    total_cost = 0
    while len(free) != 0:
        k += 1
        R_k = []
        load_k = 0
        cost_k = 0
        i = DEPOT
        dj = 0
        while len(free) != 0 and dj != float('Inf'):
            uj = None
            dj = float('Inf')
            for u in free:
                if load_k + demand[u] <= CAPACITY:
                    if short_distance[i][u[0]] < dj:
                        dj = short_distance[i][u[0]]
                        uj = u
                    elif short_distance[i][u[0]] == dj and better(u, uj, rule, load_k):
                        uj = u
            if uj is not None:
                R_k.append(uj)
                free.remove(uj)
                free.remove((uj[1], uj[0]))
                load_k += demand[uj]
                cost_k += dj + cost[uj]
                i = uj[1]
        cost_k += short_distance[i][DEPOT]
        routes.append(R_k)
        loads.append(load_k)
        costs.append(cost_k)
    for c in costs:
        total_cost += c
    return routes, loads, total_cost


def print_res(OPT_route, OPT_cost):
    print('s ', end='')
    for i in range(len(OPT_route)):
        print('0', end='')
        for j in OPT_route[i]:
            print(',' + str(j).replace(' ', ''), end='')
        print(',0', end='')
        if i != len(OPT_route) - 1:
            print(',', end='')
    print()
    print('q ' + str(OPT_cost))


def flipping(original_routes, original_costs, original_load):
    routes = copy.deepcopy(original_routes)
    costs = original_costs
    for i in range(len(routes)):
        # 如果这条路线只执行一个任务，翻不翻转无所谓
        if len(routes[i]) == 1:
            continue
        for j in range(len(routes[i])):
            # 每次路径开始元素
            if j == 0:
                origin_cost = short_distance[DEPOT][routes[i][j][0]] + short_distance[routes[i][j][1]][
                    routes[i][j + 1][0]]
                new_cost = short_distance[DEPOT][routes[i][j][1]] + short_distance[routes[i][j][0]][routes[i][j + 1][0]]
            # 路径结尾元素
            elif j == len(routes[i]) - 1:
                origin_cost = short_distance[routes[i][j - 1][1]][routes[i][j][0]] + short_distance[routes[i][j][1]][
                    DEPOT]
                new_cost = short_distance[routes[i][j - 1][1]][routes[i][j][1]] + short_distance[routes[i][j][0]][DEPOT]
            # 中间元素
            else:
                origin_cost = short_distance[routes[i][j - 1][1]][routes[i][j][0]] + short_distance[routes[i][j][1]][
                    routes[i][j + 1][0]]
                new_cost = short_distance[routes[i][j - 1][1]][routes[i][j][1]] + short_distance[routes[i][j][0]][
                    routes[i][j + 1][0]]

            if origin_cost > new_cost:
                pair = routes[i][j]
                new_pair = (pair[1], pair[0])
                routes[i][j] = new_pair
                costs -= (origin_cost - new_cost)
    return routes, costs, original_load


def self_insertion(OPT_route, OPT_cost, OPT_load):
    routes = copy.deepcopy(OPT_route)
    # 选哪条route
    route_index = np.random.randint(0, len(routes))
    route = routes[route_index]
    # 此路径的任务只有一个， 跳过此环节
    if len(route) == 1:
        return OPT_route, OPT_cost, OPT_load
    task_idx = np.random.randint(0, len(route))
    change_to_idx = np.random.randint(0, len(route))  # 需要插入的位置
    # 随机到插入自己的位置，跳过此环节
    if task_idx == change_to_idx:
        return OPT_route, OPT_cost, OPT_load

    task = route[task_idx]
    # 找到old_start，old_end
    # 如果开头是仓库
    if task_idx == 0:
        old_start = DEPOT
    else:
        old_start = route[task_idx - 1][1]
    # 如果结尾是仓库
    if task_idx == len(route) - 1:
        old_end = DEPOT
    else:
        old_end = route[task_idx + 1][0]

    # 先移动再找new_start new_end 否则会跟选中节点的先后顺序有关很难确定
    route.remove(task)
    route.insert(change_to_idx, task)

    # 找到new_start，new_end
    if change_to_idx == 0:
        new_start = DEPOT
    else:
        new_start = route[change_to_idx - 1][1]
    if change_to_idx == len(route) - 1:
        new_end = DEPOT
    else:
        new_end = route[change_to_idx + 1][0]

    # 算cost,只需要算变动的地方即可，其他不变
    new_cost = short_distance[old_start][old_end] + short_distance[new_start][task[0]] + short_distance[task[1]][
        new_end]
    old_cost = short_distance[old_start][task[0]] + short_distance[task[1]][old_end] + short_distance[new_start][
        new_end]

    if old_cost > new_cost:
        after_cost = OPT_cost + new_cost - old_cost
        return routes, after_cost, OPT_load
    return OPT_route, OPT_cost, OPT_load

def swap(OPT_route, OPT_cost, OPTLoad):
    # 如果一共只有一条线路，跳过此环节
    if len(OPT_route) == 1:
        return OPT_route, OPT_cost, OPTLoad
    routes = copy.deepcopy(OPT_route)
    loads = copy.deepcopy(OPTLoad)
    r1_index = np.random.randint(0, len(routes))
    r2_index = r1_index
    while r1_index == r2_index:
        r2_index = np.random.randint(0, len(routes))

    route1 = routes[r1_index]
    route2 = routes[r2_index]
    cnt = 0
    while cnt < len(route1) + len(route2):
        cnt += 1
        swap1_index = np.random.randint(0, len(route1))
        swap2_index = np.random.randint(0, len(route2))
        task1 = route1[swap1_index]
        task2 = route2[swap2_index]
        if swap1_index == 0:
            start1 = DEPOT
        else:
            start1 = route1[swap1_index - 1][1]
        if swap1_index == len(route1) - 1:
            end1 = DEPOT
        else:
            end1 = route1[swap1_index + 1][0]

        if swap2_index == 0:
            start2 = DEPOT
        else:
            start2 = route2[swap2_index - 1][1]
        if swap2_index == len(route2) - 1:
            end2 = DEPOT
        else:
            end2 = route2[swap2_index + 1][0]
        demand1 = OPTLoad[r1_index] - demand[task1] + demand[task2]
        demand2 = OPTLoad[r2_index] - demand[task2] + demand[task1]
        # 判断能否进行交换
        if (demand1 <= CAPACITY and demand2 <= CAPACITY):
            break

    if cnt >= len(route1) + len(route2):
        return OPT_route, OPT_cost, OPTLoad

    new_cost = short_distance[start1][task2[0]] + short_distance[task2[1]][end1] \
               + short_distance[start2][task1[0]] + short_distance[task1[1]][end2]
    old_cost = short_distance[start1][task1[0]] + short_distance[task1[1]][end1] \
               + short_distance[start2][task2[0]] + short_distance[task2[1]][end2]

    if old_cost > new_cost:
        after_cost = OPT_cost + new_cost - old_cost
        route1.remove(task1)
        route1.insert(swap1_index, task2)
        route2.remove(task2)
        route2.insert(swap2_index, task1)
        loads[r1_index] = demand1
        loads[r2_index] = demand2
        return routes, after_cost, loads
    else:
        return OPT_route, OPT_cost, OPTLoad


def single_insertion(OPT_route, OPT_cost, OPT_load):
    routes = copy.deepcopy(OPT_route)
    loads = copy.deepcopy(OPT_load)
    r1_index = np.random.randint(0, len(routes))
    route1 = routes[r1_index]

    from_task_idx = np.random.randint(0, len(route1))
    from_task = route1[from_task_idx]
    from_task_demand = demand[from_task]

    can_change_route = []
    change_route_index = {}  # 记录可交换的路径index
    index = 0
    for i in range(len(routes)):
        if i != r1_index and OPT_load[i] + from_task_demand <= CAPACITY:
            can_change_route.append(routes[i])
            change_route_index[index] = i
            index += 1
    # 如果没有符合要求的插入路径
    if index == 0:
        return OPT_route, OPT_cost, OPT_load

    r2_index = np.random.randint(0, len(can_change_route))
    route2 = can_change_route[r2_index]
    insert_index = np.random.randint(0, len(route2))

    if from_task_idx != 0:
        old_start = route1[from_task_idx - 1][1]
    else:
        old_start = DEPOT
    if from_task_idx != len(route1) - 1:
        old_end = route1[from_task_idx + 1][0]
    else:
        old_end = DEPOT

    route1.remove(from_task)
    route2.insert(insert_index, from_task)

    if insert_index != 0:
        new_start = route2[insert_index - 1][1]
    else:
        new_start = DEPOT
    if insert_index != len(route2) - 1:
        new_end = route2[insert_index + 1][0]
    else:
        new_end = DEPOT

    new_cost = short_distance[old_start][old_end] + short_distance[new_start][from_task[0]] + \
               short_distance[from_task[1]][new_end]
    old_cost = short_distance[old_start][from_task[0]] + short_distance[from_task[1]][old_end] + \
               short_distance[new_start][new_end]

    if old_cost > new_cost:
        after_cost = OPT_cost + new_cost - old_cost
        loads[r1_index] = OPT_load[r1_index] - demand[from_task]
        loads[change_route_index[r2_index]] = OPT_load[change_route_index[r2_index]] + demand[from_task]
        return routes, after_cost, loads
    else:
        return OPT_route, OPT_cost, OPT_load


def double_insertion(OPT_route, OPT_cost, OPT_load):
    routes = copy.deepcopy(OPT_route)
    loads = copy.deepcopy(OPT_load)
    r1_index = np.random.randint(0, len(routes))
    route1 = routes[r1_index]
    # 如果选到的路只有一个节点
    if len(route1) == 1:
        return OPT_route, OPT_cost, OPT_load

    from_task_idx = np.random.randint(0, len(route1) - 1)
    from_task = (route1[from_task_idx], route1[from_task_idx + 1])
    from_task_demand = demand[from_task[0]] + demand[from_task[1]]

    can_change_route = []
    change_route_index = {}
    index = 0
    for i in range(len(routes)):
        if i != r1_index and OPT_load[i] + from_task_demand <= CAPACITY:
            can_change_route.append(routes[i])
            change_route_index[index] = i
            index += 1
    if index == 0:
        return OPT_route, OPT_cost, OPT_load

    r2_index = np.random.randint(0, len(can_change_route))
    route2 = can_change_route[r2_index]
    insert_index = np.random.randint(0, len(route2))
    if from_task_idx == 0:
        old_start = DEPOT
    else:
        old_start = route1[from_task_idx - 1][1]
    if from_task_idx == len(route1) - 2:
        old_end = DEPOT
    else:
        old_end = route1[from_task_idx + 2][0]

    route1.remove(from_task[0])
    route1.remove(from_task[1])
    route2.insert(insert_index, from_task[1])
    route2.insert(insert_index, from_task[0])

    if insert_index != 0:
        new_start = route2[insert_index - 1][1]
    else:
        new_start = DEPOT
    if insert_index != len(route2) - 2:
        new_end = route2[insert_index + 2][0]
    else:
        new_end = DEPOT

    new_cost = short_distance[old_start][old_end] + short_distance[new_start][from_task[0][0]] + \
               short_distance[from_task[1][1]][new_end]
    old_cost = short_distance[old_start][from_task[0][0]] + short_distance[from_task[1][1]][old_end] + \
               short_distance[new_start][new_end]

    if old_cost > new_cost:
        after_cost = OPT_cost + new_cost - old_cost
        loads[r1_index] = OPT_load[r1_index] - demand[from_task[0]] - demand[from_task[1]]
        loads[change_route_index[r2_index]] = OPT_load[change_route_index[r2_index]] + demand[from_task[0]] + demand[from_task[1]]
        return routes, after_cost, loads
    else:
        return OPT_route, OPT_cost, OPT_load


def single_2_opt(OPT_route, OPT_cost, OPT_load):
    routes = copy.deepcopy(OPT_route)
    r_index = np.random.randint(0, len(routes))
    route = routes[r_index]
    if len(route) <= 3:
        return OPT_route, OPT_cost, OPT_load
    split_index1 = np.random.randint(1, len(route) / 2)  # 在Index前面断开
    split_index2 = np.random.randint(len(route) / 2, len(route))
    start1 = route[split_index1 - 1][1]
    end1 = route[split_index1][0]
    start2 = route[split_index2 - 1][1]
    end2 = route[split_index2][0]

    old_cost = short_distance[start1][end1] + short_distance[start2][end2]
    new_cost = short_distance[start1][start2] + short_distance[end1][end2]

    if old_cost > new_cost:
        after_cost = OPT_cost + new_cost - old_cost
        for k in range(split_index1, split_index2):
            task = route[split_index2 - 1]
            route.remove(task)
            route.insert(k, (task[1], task[0]))
        return routes, after_cost, OPT_load
    return OPT_route, OPT_cost, OPT_load


def cross_2_opt(OPT_route, OPT_cost, OPT_load):
    if len(OPT_route) == 1:
        return OPT_route, OPT_cost, OPT_load
    routes = copy.deepcopy(OPT_route)
    loads = copy.deepcopy(OPT_load)
    r1_index = np.random.randint(0, len(routes))
    r2_index = r1_index
    while r2_index == r1_index:
        r2_index = np.random.randint(0, len(routes))
    route1 = routes[r1_index]
    route2 = routes[r2_index]

    cnt = 0
    demand1, demand2, demand3, demand4 = 0, 0, 0, 0
    while cnt < len(route1) + len(route2):
        split1 = np.random.randint(1, len(route1))
        split2 = np.random.randint(1, len(route2))
        demand1, demand2, demand3, demand4 = 0, 0, 0, 0
        for k in range(0, split1):
            demand1 += demand[route1[k]]
        for k in range(split1, len(route1)):
            demand2 += demand[route1[k]]
        for k in range(0, split2):
            demand3 += demand[route2[k]]
        for k in range(split2, len(route2)):
            demand4 += demand[route2[k]]
        cnt += 1
        if (demand1 + demand3 <= CAPACITY and demand2 + demand4 <= CAPACITY) or (
                demand1 + demand4 <= CAPACITY and demand2 + demand3 <= CAPACITY):
            break
    if cnt >= len(route1) + len(route2):
        return OPT_route, OPT_cost, OPT_load
    child_cost1 = float('Inf')
    child_cost2 = float('Inf')
    if demand1 + demand4 <= CAPACITY and demand2 + demand3 <= CAPACITY:
        old_cost = short_distance[route1[split1 - 1][1]][route1[split1][0]] + short_distance[route2[split2 - 1][1]][
            route2[split2][0]]
        new_cost = short_distance[route1[split1 - 1][1]][route2[split2][0]] + short_distance[route2[split2 - 1][1]][
            route1[split1][0]]
        child_cost1 = OPT_cost + new_cost - old_cost

    if demand1 + demand3 <= CAPACITY and demand2 + demand4 <= CAPACITY:
        old_cost = short_distance[route1[split1 - 1][1]][route1[split1][0]] + short_distance[route2[split2 - 1][1]][
            route2[split2][0]]
        new_cost = short_distance[route1[split1 - 1][1]][route2[split2 - 1][1]] + short_distance[route2[split2][0]][
            route1[split1][0]]
        child_cost2 = OPT_cost + new_cost - old_cost

    if child_cost1 < child_cost2 and child_cost1 < OPT_cost:
        after_cost = child_cost1
        new_route1 = route1[:split1] + route2[split2:]
        new_route2 = route2[:split2] + route1[split1:]
        loads[r1_index] = demand1 + demand4
        loads[r2_index] = demand2 + demand3
        routes[r1_index] = new_route1
        routes[r2_index] = new_route2
        # print("进来过1")
        return routes, after_cost, loads
    if child_cost2 <= child_cost1 and child_cost2 < OPT_cost:
        after_cost = child_cost2
        reverse1 = route1[split1:]
        for k in range(len(reverse1)):
            task = reverse1[k]
            reverse1[k] = (task[1], task[0])
        reverse1_re = list(reversed(reverse1))

        reverse2 = route2[:split2]
        for k in range(len(reverse2)):
            task = reverse2[k]
            reverse2[k] = (task[1], task[0])
        reverse2_re = list(reversed(reverse2))

        new_route1 = route1[:split1] + reverse2_re
        new_route2 = reverse1_re + route2[split2:]
        loads[r1_index] = demand1 + demand3
        loads[r2_index] = demand2 + demand4
        routes[r1_index] = new_route1
        routes[r2_index] = new_route2
        # print("进来过2")
        return routes, after_cost, loads
    return OPT_route, OPT_cost, OPT_load


if __name__ == '__main__':
    # input_file = 'E:\大三上\AI\project2_carp\CARP\CARP_samples\\val7A.dat'
    # terminal_time = 10
    # random_seed = 0
    input_file = sys.argv[1]
    terminal_time = int(sys.argv[3])
    random_seed = int(sys.argv[5])
    random_seed = random_seed % (2 ** 32)
    np.random.seed(random_seed)

    start_time = time.time()
    read_info(input_file)
    Floyd()

    phase3_time = min(30, int(terminal_time / 3))
    phase1_time = (terminal_time - phase3_time) / 5 * 3
    phase2_time = terminal_time - phase3_time

    best_cost = float('Inf')
    best_route = []
    best_load = []
    phase1_weight = [0, 0.05, 0.25, 0.45, 0.65, 0.75]
    phase2_weight = [0, 0.1, 0.35, 0.55, 0.75, 0.85]
    phase3_weight = [0.02, 0.22, 0.47, 0.62, 0.82, 0.85]
    parents = []
    # 第一阶段 变异率较大
    for rule in range(5):
        cur_route, cur_load, cur_cost = path_scanning(rule)
        parents.append((cur_cost, cur_route, cur_load))
        # heapq.heappush(min_heap, (cur_cost, cur_route, cur_load))
    while time.time() - start_time < phase1_time - 23:
        loop_time = time.time()
        while time.time() - loop_time < 20:
            change_index = np.random.randint(0, len(parents))
            cur_change = parents[change_index]
            cur_cost = cur_change[0]
            cur_route = cur_change[1]
            cur_load = cur_change[2]
            r = np.random.rand()
            if r < phase1_weight[0]:
                change_route, change_cost, change_load = flipping(cur_route, cur_cost, cur_load)
            elif r < phase1_weight[1]:
                change_route, change_cost, change_load = self_insertion(cur_route, cur_cost, cur_load)
            elif r < phase1_weight[2]:
                change_route, change_cost, change_load = single_insertion(cur_route, cur_cost, cur_load)
            elif r < phase1_weight[3]:
                change_route, change_cost, change_load = double_insertion(cur_route, cur_cost, cur_load)
            elif r < phase1_weight[4]:
                change_route, change_cost, change_load = swap(cur_route, cur_cost, cur_load)
            elif r < phase1_weight[5]:
                change_route, change_cost, change_load = single_2_opt(cur_route, cur_cost, cur_load)
            else:
                change_route, change_cost, change_load = cross_2_opt(cur_route, cur_cost, cur_load)
            if change_cost < cur_cost:
                parents.append((change_cost, change_route, change_load))
        new_parents = heapq.nsmallest(10, parents, key=lambda s: s[0])
        parents = new_parents

    # 第二阶段 变异率较小 取向局部最优
    while time.time() - start_time < phase2_time - 23:
        loop_time = time.time()
        while time.time() - loop_time < 20:
            change_index = np.random.randint(0, len(parents))
            cur_change = parents[change_index]
            cur_cost = cur_change[0]
            cur_route = cur_change[1]
            cur_load = cur_change[2]
            r = np.random.rand()
            if r < phase2_weight[0]:
                change_route, change_cost, change_load = flipping(cur_route, cur_cost, cur_load)
            elif r < phase2_weight[1]:
                change_route, change_cost, change_load = self_insertion(cur_route, cur_cost, cur_load)
            elif r < phase2_weight[2]:
                change_route, change_cost, change_load = single_insertion(cur_route, cur_cost, cur_load)
            elif r < phase2_weight[3]:
                change_route, change_cost, change_load = double_insertion(cur_route, cur_cost, cur_load)
            elif r < phase2_weight[4]:
                change_route, change_cost, change_load = swap(cur_route, cur_cost, cur_load)
            elif r < phase2_weight[5]:
                change_route, change_cost, change_load = single_2_opt(cur_route, cur_cost, cur_load)
            else:
                change_route, change_cost, change_load = cross_2_opt(cur_route, cur_cost, cur_load)
            if change_cost < cur_cost:
                parents.append((change_cost, change_route, change_load))
        new_parents = heapq.nsmallest(10, parents, key=lambda s: s[0])
        parents = new_parents

    res = heapq.nsmallest(1, parents, key=lambda s: s[0])[0]
    best_route, best_cost, best_load = res[1], res[0], res[2]
    # 第三阶段 选取最优的进行优化
    while time.time() - start_time < terminal_time - 5:
        r = np.random.rand()
        if r < phase3_weight[0]:
            change_route, change_cost, change_load = flipping(best_route, best_cost, best_load)
        elif r < phase3_weight[1]:
            change_route, change_cost, change_load = self_insertion(best_route, best_cost, best_load)
        elif r < phase3_weight[2]:
            change_route, change_cost, change_load = single_insertion(best_route, best_cost, best_load)
        elif r < phase3_weight[3]:
            change_route, change_cost, change_load = double_insertion(best_route, best_cost, best_load)
        elif r < phase3_weight[4]:
            change_route, change_cost, change_load = swap(best_route, best_cost, best_load)
        elif r < phase3_weight[5]:
            change_route, change_cost, change_load = single_2_opt(best_route, best_cost, best_load)
        else:
            change_route, change_cost, change_load = cross_2_opt(best_route, best_cost, best_load)
        if change_cost < best_cost:
            best_route = change_route
            best_cost = change_cost
            best_load = change_load
    best_route, best_cost, best_load = flipping(best_route, best_cost, best_load)
    print_res(best_route, best_cost)

# 优的才加Heap
#  phase3_time = min(40,int(terminal_time/3))
#     phase1_time = (terminal_time-phase3_time)/2 10
#     phase2_time = (terminal_time-phase3_time)  10
#     288	316	5341	3849	311	428	182
# 288	323	5371	3849	311	428	182

# 优的才加Heap
#  phase3_time = min(40,int(terminal_time/3))
#     phase1_time = (terminal_time-phase3_time)/2 20
#     phase2_time = (terminal_time-phase3_time)  10
#     288	316	5397	3849	311	428	182

# 排除
# phase3_time = min(40,int(terminal_time/3))
#     phase1_time = (terminal_time-phase3_time)/2 10  不优的也加heap
#     phase2_time = (terminal_time-phase3_time)  10   优的才加heap
#   288	316	5644	3965	323	428	186

# 优的才加Heap
#  phase3_time = min(40,int(terminal_time/3))
#     phase1_time = (terminal_time-phase3_time)/3*2 10
#     phase2_time = (terminal_time-phase3_time)  10
#     288	323	5453	3837	311	403	186

# 优的才加Heap
#  phase3_time = min(40,int(terminal_time/3))
#     phase1_time = (terminal_time-phase3_time)/5*3 10
#     phase2_time = (terminal_time-phase3_time)  10
#     288	316	5332	3919	311	428	186

# phase3_time = min(30, int(terminal_time / 3))
# phase1_time = (terminal_time - phase3_time) / 2 10
# phase2_time = terminal_time - phase3_time
# 288	316	5294	3837	311	428	186

# phase3_time = min(30, int(terminal_time / 3))
# phase1_time = (terminal_time - phase3_time) / 5 * 3  10
# phase2_time = terminal_time - phase3_time
# 288	331	5268	3749	305	418	186
# 288	316	5348	3914	304	428	182

# phase3_time = min(20, int(terminal_time / 3))
# phase1_time = (terminal_time - phase3_time) / 5 * 3  10
# phase2_time = terminal_time - phase3_time
# 288	316	5318	3965	311	428	186

#    phase1_weight = [0, 0.05, 0.25, 0.45, 0.65, 0.75]
#     phase2_weight = [0, 0.1, 0.35, 0.55, 0.75, 0.85]
#     phase3_weight = [0.02, 0.22, 0.47, 0.62, 0.82, 0.85]
# 288	329	5292	3861	311	428	186
# 288	316	5196	3849	311	428	186

   # phase1_weight = [0, 0.05, 0.15, 0.35, 0.55, 0.75]
   #  phase2_weight = [0, 0.2, 0.4, 0.6, 0.8, 0.85]
   #  phase3_weight = [0.02, 0.22, 0.47, 0.62, 0.82, 0.85]
# 288	316	5411	3708	305	428	182
# 288	323	5520	3977	311	416	182
