import time
import gurobipy as gp
from gurobipy import GRB
from Parameter import *

def look_head_single_dynamic_policy(para, L, dynamic_threshold):
    print(f'---look_head_single_dynamic_L={L}_tau={dynamic_threshold}---')
    T = para['T']
    V = para['V']
    F = para['F']
    unit_consumption = para['unit_consumption']
    unit_charging = para['unit_charging']
    max_electric = para['max_electric']
    min_electric = para['min_electric']
    total_node = para['total_node']
    vehicle_origin = para['vehicle_origin']
    vehicle_destination = para['vehicle_destination']
    node_out_arc = para['node_out_arc']
    node_in_arc = para['node_in_arc']
    operation_arc = para['operation_arc']
    waiting_arc = para['waiting_arc']
    charging_arc = para['charging_arc']
    charging_arc_cost = para['charging_arc_cost']
    charging_arc_p0 = para['charging_arc_p0']
    charging_arc_p1 = para['charging_arc_p1']
    charging_arc_p0_cost = para['charging_arc_p0_cost']
    charging_arc_p1_cost = para['charging_arc_p1_cost']
    poll_arc = para['poll_arc']
    up_out_node = para['up_out_node']
    down_out_node = para['down_out_node']
    time_stamp = para['time_stamp']
    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    node_time = {n: n if n < T else n - T for n in total_node}
    time_pair = [(time_stamp[t], time_stamp[t + 1]) for t in range(len(time_stamp) - 1)]
    start = time.time()
    vehicle_task_volume = {v: 0 for v in range(V)}
    X_value = {(v, i, j): 0 for v in range(V) for i, j in vehicle_arc}
    unfinished_task = [(i, j) for i, j in operation_arc]
    charging_count = {t: {0: [], 1: []} for t in range(T)}
    finished_task = set()
    vehicle_fleet = 0
    direction = 2
    node_stop = {n: 0 if n < T else 1 for n in total_node}

    for v in range(V):
        debt_double = {t: sum(1 for i, j in unfinished_task if node_time[i] <= t < node_time[j]) for t in range(T)}
        debt = {(t, d): sum(1 for i, j in unfinished_task if node_time[i] <= t < node_time[j] if node_stop[i] == d) for t in range(T) for d in range(direction)}
        if len(unfinished_task) <= dynamic_threshold:
            L_adjust = max(debt_double.values())
            L = max(L, L_adjust)
        if unfinished_task:
            vehicle_fleet += 1
            model = gp.Model('vehicle')
            X = model.addVars(L, vehicle_arc, vtype=GRB.BINARY, name='X')
            Z = model.addVars(L, time_stamp, lb=min_electric, ub=max_electric, vtype=GRB.CONTINUOUS, name='Z')
            S = model.addVars(L+1, unfinished_task, vtype=GRB.BINARY, name='S')
            H_0 = model.addVars(L+1, time_stamp, vtype=GRB.INTEGER, name='H_0')
            H_1 = model.addVars(L+1, time_stamp, vtype=GRB.INTEGER, name='H')
            for i,j in unfinished_task:
                model.addConstr(S[0, i, j] == 1)
            for l in range(L):
                for i, j in unfinished_task:
                    model.addConstr(S[l+1, i, j] == S[l, i, j] - X[l, i, j])
            for t in time_stamp:
                model.addConstr(H_0[0, t] == len(charging_count[t][0]))
                model.addConstr(H_1[0, t] == len(charging_count[t][1]))
            for l in range(L):
                for t in time_stamp:
                    model.addConstr(H_0[l+1, t] == H_0[l, t] + gp.quicksum(X[l, i, j] for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j]))
                    model.addConstr(H_1[l+1, t] == H_1[l, t] + gp.quicksum(X[l, i, j] for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j]))
            for l in range(L):
                for i, j in finished_task:
                    X[l, i, j].ub = 0
            obj1 = gp.quicksum(
                debt[t, d] * X[l, i, j] for l in range(L) for d in range(direction) for t in range(T) for i, j in unfinished_task
                if node_time[i] <= t < node_time[j] if node_stop[i] == d)
            balance_coefficient_0 = 1
            balance_coefficient_1 = 1
            obj2 = gp.quicksum(len(charging_count[t][0]) * X[l, i, j] for l in range(L) for t in range(T) for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j])
            obj3 = gp.quicksum(len(charging_count[t][1]) * X[l, i, j] for l in range(L) for t in range(T) for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j])
            obj4 = gp.quicksum(charging_arc_cost[i,j] * X[l, i, j] for l in range(L) for i, j in charging_arc)
            model.setObjective(obj1 - balance_coefficient_0 * obj2 - balance_coefficient_1 * obj3 - obj4, GRB.MAXIMIZE)

            for l in range(L):
                for i, j in unfinished_task:
                    model.addConstr(X[l, i, j] <= S[l, i, j])
            model.addConstrs((gp.quicksum(
                debt[t, d] * X[l, i, j] for d in range(direction) for t in range(T) for i, j in unfinished_task
                if node_time[i] <= t < node_time[j] if node_stop[i] == d) >= gp.quicksum(
                debt[t, d] * X[l + 1, i, j] for d in range(direction) for t in range(T) for i, j in unfinished_task
                if node_time[i] <= t < node_time[j] if node_stop[i] == d) for l in range(L - 1)), name='look_head')

            model.addConstrs(
                (gp.quicksum(X[l, m, n] for m, n in node_out_arc[vehicle_origin[v]]) -
                 gp.quicksum(X[l, m, n] for m, n in node_in_arc[vehicle_origin[v]]) == 1
                 for l in range(L)),
                name='origin_flow'
            )

            model.addConstrs(
                (gp.quicksum(X[l, m, n] for m, n in node_out_arc[vehicle_destination[v]]) -
                 gp.quicksum(X[l, m, n] for m, n in node_in_arc[vehicle_destination[v]]) == -1
                 for l in range(L)),
                name='destination_flow'
            )

            model.addConstrs(
                (gp.quicksum(X[l, m, n] for m, n in node_out_arc[i]) -
                 gp.quicksum(X[l, m, n] for m, n in node_in_arc[i]) == 0
                 for i in total_node if i != vehicle_origin[v] and i != vehicle_destination[v]
                 for l in range(L)),
                name='inter_flow'
            )
            for l in range(L):
                for t in time_stamp:
                    model.addConstr(gp.quicksum(X[l, i, j] for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j]) <= F - H_0[l, t])
                    model.addConstr(gp.quicksum(X[l, i, j] for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j]) <= F - H_1[l, t])

            model.addConstrs((Z[l, time_stamp[0]] == max_electric for l in range(L)), name='e_start')

            for l in range(L):
                for to, td in time_pair:
                    consumption = gp.quicksum(X[l, i, j] * unit_consumption * (td - to) for i, j in operation_arc
                                              if node_time[i] <= to < td <= node_time[j])
                    charging = gp.quicksum(X[l, i, j] * unit_charging * (td - to) for i, j in charging_arc if
                                           node_time[i] <= to < td <= node_time[j])
                    model.addConstr(Z[l, td] == Z[l, to] - consumption + charging, name=f'transfer({to},{td})')

            model.addConstrs((X[l, 2 * T, T] == X[l, up_out_node, 2 * T + 1] for l in range(L)), name='special1')
            model.addConstrs((X[l, 2 * T, 0] == X[l, down_out_node, 2 * T + 1] for l in range(L)), name='special2')

            model.setParam('OutputFlag', 0)
            model.setParam('Threads', 20)
            model.optimize()

            if obj1.getValue() == 0:
                unfinished_task_copy = [(i, j) for i, j in unfinished_task]
                for i, j in unfinished_task_copy:
                    X_value[v, i, j] = 1
                    v += 1
                    vehicle_fleet += 1
                    unfinished_task.remove((i, j))
                vehicle_fleet -= 1
                break

            now_task = sum(X[l, i, j].x > 0.5 for l in range(L) for i, j in unfinished_task)

            if now_task == len(unfinished_task):
                used_vehicle_count = 0
                for l in range(L):
                    has_operation = any(
                        X[l, i, j].x >= 0.5
                        for (i, j) in operation_arc)

                    if not has_operation:
                        continue

                    vehicle_id = v + used_vehicle_count

                    for i, j in vehicle_arc:
                        if X[l, i, j].x >= 0.5:
                            X_value[vehicle_id, i, j] = 1

                            if (i, j) in operation_arc:
                                vehicle_task_volume[vehicle_id] += 1
                                finished_task.add((i, j))
                                unfinished_task.remove((i, j))
                                
                    used_vehicle_count += 1

                vehicle_fleet += used_vehicle_count - 1
                break

            list_num = [sum(X[l, i, j].x for i, j in unfinished_task) for l in range(L)]
            list_l = [
                sum(debt[t, d] * X[l, i, j].x for d in range(direction) for t in range(T) for i, j in unfinished_task if
                    node_time[i] <= t < node_time[j] if node_stop[i] == d)
                for l in range(L)
            ]
            max_l_index = 0
            max_l_value = 0
            for i in range(len(list_l)):
                if list_l[i] > max_l_value:
                    max_l_value = list_l[i]
                    max_l_index = i
                if list_l[i] == max_l_value and list_num[i] > list_num[max_l_index]:
                    max_l_value = list_l[i]
                    max_l_index = i

            for i, j in vehicle_arc:
                if X[max_l_index, i, j].x >= 0.5:
                    X_value[v, i, j] = 1
                    if (i, j) in operation_arc:
                        vehicle_task_volume[v] += 1
                        finished_task.add((i, j))
                        unfinished_task.remove((i, j))

            for i, j in charging_arc_p0:
                if X_value[v, i, j] == 1:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][0].append(v)

            for i, j in charging_arc_p1:
                if X_value[v, i, j] == 1:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][1].append(v)

    total_running_time = time.time() - start
    return {'X_value': X_value, 'fleet_size': vehicle_fleet, 'running_time': total_running_time}
