import time
import gurobipy as gp
from gurobipy import GRB
import numpy as np
from Parameter import *

def greedy_policy(para):
    print('----greedy----')
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
    poll_arc = para['poll_arc']
    up_out_node = para['up_out_node']
    down_out_node = para['down_out_node']
    time_stamp = para['time_stamp']

    vehicle_task_volume = {v: 0 for v in range(V)}
    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    node_time = {n: n if n < T else n - T for n in total_node}
    time_pair = [(time_stamp[t], time_stamp[t + 1]) for t in range(len(time_stamp) - 1)]
    start = time.time()
    X_value = {(v, i, j): 0 for v in range(V) for i, j in vehicle_arc}
    unfinished_task = [(i, j) for i, j in operation_arc]
    charging_count = {t: {0: [], 1: []} for t in range(T)}
    finished_task = []
    vehicle_fleet = 0
    for v in range(V):
        if len(unfinished_task) > 0:
            vehicle_fleet += 1
            model = gp.Model('greedy')
            X = model.addVars(vehicle_arc, vtype=GRB.BINARY, name='X')
            Z = model.addVars(time_stamp, lb=min_electric, ub=max_electric, vtype=GRB.CONTINUOUS, name='Z')
            for i, j in finished_task:
                X[i, j].ub = 0
            obj = gp.quicksum(X[i, j] for i, j in unfinished_task)
            obj1 = gp.quicksum(charging_arc_cost[i,j] * X[i, j] for i, j in charging_arc)
            model.setObjective(obj - obj1, GRB.MAXIMIZE)
            model.addConstr(gp.quicksum(X[m, n] for m, n in node_out_arc[vehicle_origin[v]]) - gp.quicksum(
                X[m, n] for m, n in node_in_arc[vehicle_origin[v]]) == 1, name='origin_flow')
            model.addConstr(gp.quicksum(X[m, n] for m, n in node_out_arc[vehicle_destination[v]]) - gp.quicksum(
                X[m, n] for m, n in node_in_arc[vehicle_destination[v]]) == -1, name='destination_flow')
            model.addConstrs(
                (gp.quicksum(X[m, n] for m, n in node_out_arc[i]) - gp.quicksum(X[m, n] for m, n in node_in_arc[i]) == 0
                 for i in total_node if i != vehicle_origin[v] and i != vehicle_destination[v]), name='inter_flow')

            for t in time_stamp:
                model.addConstr(gp.quicksum(X[i, j] for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][0]))
                model.addConstr(gp.quicksum(X[i, j] for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][1]))

            model.addConstr(Z[time_stamp[0]] == max_electric, name='e_start')

            for to, td in time_pair:
                consumption = gp.quicksum(X[i, j] * unit_consumption * (td - to) for i, j in operation_arc if node_time[i] <= to < node_time[j])
                charging = gp.quicksum(X[i, j] * unit_charging * (td - to) for i, j in charging_arc if node_time[i] <= to < node_time[j])
                model.addConstr(Z[td] == Z[to] - consumption + charging, name='transfer({},{})'.format(to, td))

            model.addConstr(X[2 * T, T] == X[up_out_node, 2 * T + 1], name='special1')
            model.addConstr(X[2 * T, 0] == X[down_out_node, 2 * T + 1], name='special2')

            model.setParam('OutputFlag', 1)
            model.optimize()
            if obj.getValue() == 0:
                unfinished_task_copy = [(i, j) for i, j in unfinished_task]
                for i, j in unfinished_task_copy:
                    X_value[v, i, j] = 1
                    v += 1
                    vehicle_fleet += 1
                    unfinished_task.remove((i, j))
                vehicle_fleet -= 1
                break
            for i, j in vehicle_arc:
                if X[i, j].x >= 0.5:
                    X_value[v, i, j] = 1
                if X[i, j].x >= 0.5 and (i, j) not in waiting_arc:
                    if (i, j) in operation_arc:
                        vehicle_task_volume[v] += 1
                        finished_task.append((i, j))
                        unfinished_task.remove((i, j))
            for i,j in charging_arc_p0:
                if X[i, j].x >= 0.5:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][0].append(v)
            for i,j in charging_arc_p1:
                if X[i, j].x >= 0.5:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][1].append(v)
    total_running_time = time.time() - start

    return {'X_value': X_value, 'fleet_size': vehicle_fleet, 'running_time': total_running_time}