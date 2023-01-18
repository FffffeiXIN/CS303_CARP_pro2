import sys
import copy
import numpy as np

VERTICES_NUM_plus1 = 0
DEPOT = 0
CAPACITY = 0
demand = {}
cost = {}
short_distance = []
weight = [0.05, 0.20, 0.45, 0.75, 1]


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
    free = [item for item in demand.keys()]
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
            dj = float('Inf')
            uj = None
            for u in free:
                if load_k + demand[u] <= CAPACITY:
                    if short_distance[i][u[0]] < dj:
                        dj = short_distance[i][u[0]]
                        uj = u
                    elif short_distance[i][u[0]] == dj and better(u, uj, rule, load_k):
                        uj = u
            if uj != None:
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


def flipping(routes, costs):
    # routes = np.copy(original_routes)
    for i in range(len(routes)):  # 遍历所有弧
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
    return routes, costs


def self_insertion(OPT_route, OPT_cost):
    routes = copy.deepcopy(OPT_route)
    # 选哪条route
    route_index = np.random.randint(0, len(routes))
    route = routes[route_index]
    # 此路径的任务只有一个， 跳过此环节
    if len(route) == 1:
        return OPT_route, OPT_cost
    task_idx = np.random.randint(0, len(route))
    change_to_idx = np.random.randint(0, len(route))  # 需要插入的位置
    # 随机到插入自己的位置，跳过此环节
    if task_idx == change_to_idx:
        return OPT_route, OPT_cost

    task = route[task_idx]

    # 找到old_start，old_end
    if task_idx == 0:
        old_start = DEPOT
    else:
        old_start = route[task_idx - 1][1]
    if task_idx == len(route) - 1:
        old_end = DEPOT
    else:
        old_end = route[task_idx + 1][0]

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

    old_cost = short_distance[old_start][task[0]] + short_distance[task[1]][old_end] + short_distance[new_start][
        new_end]
    new_cost = short_distance[old_start][old_end] + short_distance[new_start][task[0]] + short_distance[task[1]][
        new_end]

    if old_cost > new_cost:
        OPT_cost -= (old_cost - new_cost)
        return routes, OPT_cost
    return OPT_route, OPT_cost


def swap(OPT_route, OPT_cost, OPT_load):
    # global OPT_route, OPT_cost, OPT_load
    # print(OPT_cost)
    routes = copy.deepcopy(OPT_route)
    r1 = np.random.randint(0, len(routes))
    r2 = r1
    while r1 == r2:
        r2 = np.random.randint(0, len(routes))
    route1 = routes[r1]
    route2 = routes[r2]
    cnt = 0
    while True:
        pos1 = np.random.randint(0, len(route1))
        pos2 = np.random.randint(0, len(route2))
        task1 = route1[pos1]
        task2 = route2[pos2]
        if pos1 == 0:
            start1 = DEPOT
        else:
            start1 = route1[pos1 - 1][1]
        if pos1 == len(route1) - 1:
            end1 = DEPOT
        else:
            end1 = route1[pos1 + 1][0]

        if pos2 == 0:
            start2 = DEPOT
        else:
            start2 = route2[pos2 - 1][1]
        if pos2 == len(route2) - 1:
            end2 = DEPOT
        else:
            end2 = route2[pos2 + 1][0]

        demand1 = OPT_load[r1] - demand[task1] + demand[task2]
        demand2 = OPT_load[r2] - demand[task2] + demand[task1]
        cnt += 1
        if (demand1 <= CAPACITY and demand2 <= CAPACITY) or cnt >= len(route1) + len(route2):
            break

    if cnt >= len(route1) + len(route2):
        # print(OPT_cost)
        return OPT_route, OPT_cost

    old_cost = short_distance[start1][task1[0]] + short_distance[task1[1]][end1] \
               + short_distance[start2][task2[0]] + short_distance[task2[1]][end2]
    new_cost = short_distance[start1][task2[0]] + short_distance[task2[1]][end1] \
               + short_distance[start2][task1[0]] + short_distance[task1[1]][end2]

    if old_cost > new_cost:
        OPT_cost += new_cost - old_cost
        route1.remove(task1)
        route1.insert(pos1, task2)
        route2.remove(task2)
        route2.insert(pos2, task1)
        OPT_load[r1] = demand1
        OPT_load[r2] = demand2
        # print(OPT_cost)
        # print(demand1)
        # print(demand2)
        return routes, OPT_cost
    else:
        # print(OPT_cost)
        return OPT_route, OPT_cost


def cross_single_insertion(OPT_route, OPT_cost, OPT_load):
    # global OPT_route, OPT_cost
    routes = copy.deepcopy(OPT_route)
    r1 = np.random.randint(0, len(routes))  # 选择一条需要提取段的子路径
    route1 = routes[r1]  # 子路径本径
    from_task_idx = np.random.randint(0, len(route1))  # 将要取出进行交换的段
    from_task = route1[from_task_idx]  # 段本段
    from_task_demand = demand[from_task]  # 该段的任务量
    route_candidate = []  # 挑选出可以插入的路线作为候选
    idx_map = {}  # 记录候选者的全局路径索引
    cnt = 0
    for i in range(len(routes)):
        if i != r1 and OPT_load[i] + from_task_demand <= CAPACITY:
            route_candidate.append(routes[i])
            idx_map[cnt] = i
            cnt = cnt + 1
    if cnt == 0:  # 没有符合的路径
        return OPT_route, OPT_cost
    r2 = np.random.randint(0, len(route_candidate))
    route2 = route_candidate[r2]
    insertion_pos = np.random.randint(0, len(route2))  # 将要插入的位置（去首去尾）
    if from_task_idx == 0:
        old_start = DEPOT
    else:
        old_start = route1[from_task_idx - 1][1]
    if from_task_idx == len(route1) - 1:
        old_end = DEPOT
    else:
        old_end = route1[from_task_idx + 1][0]

    route1.remove(from_task)
    route2.insert(insertion_pos, from_task)

    if insertion_pos == 0:
        new_start = DEPOT
    else:
        new_start = route2[insertion_pos - 1][1]
    if insertion_pos == len(route2) - 1:
        new_end = DEPOT
    else:
        new_end = route2[insertion_pos + 1][0]

    old_cost = short_distance[old_start][from_task[0]] + short_distance[from_task[1]][old_end] + \
               short_distance[new_start][new_end]
    new_cost = short_distance[old_start][old_end] + short_distance[new_start][from_task[0]] + \
               short_distance[from_task[1]][new_end]

    if old_cost > new_cost:
        OPT_cost += new_cost - old_cost
        OPT_load[r1] = OPT_load[r1] - demand[from_task]
        OPT_load[idx_map[r2]] = OPT_load[idx_map[r2]] + demand[from_task]
        return routes, OPT_cost
    else:
        return OPT_route, OPT_cost


def cross_double_insertion(OPT_route, OPT_cost, OPT_load):
    # global OPT_route, OPT_cost
    routes = copy.deepcopy(OPT_route)
    r1 = np.random.randint(0, len(routes))  # 选择一条需要提取段的子路径
    route1 = routes[r1]  # 子路径本径
    if len(route1) == 1:
        return OPT_route, OPT_cost
    from_task_idx = np.random.randint(0, len(route1) - 1)  # 将要取出进行交换的段
    from_task = (route1[from_task_idx], route1[from_task_idx + 1])  # 两个段
    from_task_demand = demand[from_task[0]] + demand[from_task[1]]  # 该段的任务量
    route_candidate = []  # 挑选出可以插入的路线作为候选
    idx_map = {}  # 记录候选者的全局路径索引
    cnt = 0
    for i in range(len(routes)):
        if i != r1 and OPT_load[i] + from_task_demand <= CAPACITY:
            route_candidate.append(routes[i])
            idx_map[cnt] = i
            cnt = cnt + 1
    if cnt == 0:  # 没有符合的路径
        return OPT_route, OPT_cost
    r2 = np.random.randint(0, len(route_candidate))
    route2 = route_candidate[r2]
    insertion_pos = np.random.randint(0, len(route2))  # 将要插入的位置（去首去尾）
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
    route2.insert(insertion_pos, from_task[1])
    route2.insert(insertion_pos, from_task[0])

    if insertion_pos == 0:
        new_start = DEPOT
    else:
        new_start = route2[insertion_pos - 1][1]
    if insertion_pos == len(route2) - 2:
        new_end = DEPOT
    else:
        new_end = route2[insertion_pos + 2][0]

    old_cost = short_distance[old_start][from_task[0][0]] + short_distance[from_task[1][1]][old_end] + \
               short_distance[new_start][new_end]
    new_cost = short_distance[old_start][old_end] + short_distance[new_start][from_task[0][0]] + \
               short_distance[from_task[1][1]][new_end]

    if old_cost > new_cost:
        OPT_cost += new_cost - old_cost
        OPT_load[r1] = OPT_load[r1] - demand[from_task[0]] - demand[from_task[1]]
        OPT_load[idx_map[r2]] = OPT_load[idx_map[r2]] + demand[from_task[0]] + demand[from_task[1]]
        return routes, OPT_cost
    else:
        return OPT_route, OPT_cost


if __name__ == '__main__':
    # input_file = 'E:\大三上\AI\project2_carp\CARP\CARP_samples\gdb10.dat'
    input_file = sys.argv[1]
    terminal_time = sys.argv[3]
    random_seed = int(sys.argv[5])
    random_seed = random_seed % (2 ** 32)
    np.random.seed(random_seed)
    read_info(input_file)
    Floyd()

    best_cost = float('Inf')
    best_route = []
    for rule in range(5):
        cur_route, cur_load, cur_cost = path_scanning(rule)
        for i in range(30000):
            r = np.random.rand()
            if r < weight[0]:
                cur_route, cur_cost = flipping(cur_route, cur_cost)
            elif r < weight[1]:
                cur_route, cur_cost = self_insertion(cur_route, cur_cost)
            elif r < weight[2]:
                cur_route, cur_cost = cross_single_insertion(cur_route, cur_cost, cur_load)
            elif r < weight[3]:
                cur_route, cur_cost = cross_double_insertion(cur_route, cur_cost, cur_load)
            elif r < weight[4]:
                cur_route, cur_cost = swap(cur_route, cur_cost, cur_load)
        if cur_cost < best_cost:
            best_route = cur_route
            best_cost = cur_cost
    print_res(best_route, best_cost)
    # 288	342	5607	3908	312	428	188
    # 到时候再加otp2吧 然后看看把五个单独先训五分钟，找到最优解之后单独优化
    # 或者按时间算代遗传筛选啥的 otp2算
    # 或许可以不在方法里cost小才找最优 可以先加代里继续训 后面在看
