import time
import gurobipy as gp
from gurobipy import GRB
from Parameter import *
import pandas as pd


def MIP_policy(para):
    print('----MIP----')
    T = para['T']
    F = para['F']
    V = para['V']
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

    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    node_time = {n: n if n < T else n - T for n in total_node}
    time_pair = [(time_stamp[t], time_stamp[t + 1]) for t in range(len(time_stamp) - 1)]
    start_time = time.time()
    X_value = {(v, i, j): 0 for v in range(V) for i, j in vehicle_arc}
    model = gp.Model('MIP')
    X = model.addVars(V, vehicle_arc, vtype=GRB.BINARY, name='X')
    Y = model.addVars(V, vtype=GRB.BINARY, name='Y')
    Z = model.addVars(V, time_stamp, lb=min_electric, ub=max_electric, vtype=GRB.CONTINUOUS, name='Z')
    obj = gp.quicksum(Y[v] for v in range(V))
    obj_1 = gp.quicksum(charging_arc_cost[i,j] * X[v, i, j] for v in range(V) for i,j in charging_arc)
    model.setObjective(obj + obj_1, GRB.MINIMIZE)
    model.addConstrs((gp.quicksum(X[v, m, n] for m, n in node_out_arc[vehicle_origin[v]]) - gp.quicksum(
        X[v, m, n] for m, n in node_in_arc[vehicle_origin[v]]) == 1 for v in range(V)))
    model.addConstrs((gp.quicksum(X[v, m, n] for m, n in node_out_arc[vehicle_destination[v]]) - gp.quicksum(
        X[v, m, n] for m, n in node_in_arc[vehicle_destination[v]]) == -1 for v in range(V)))
    model.addConstrs(
        (gp.quicksum(X[v, m, n] for m, n in node_out_arc[i]) - gp.quicksum(X[v, m, n] for m, n in node_in_arc[i]) == 0
         for v in range(V) for i in total_node if i != vehicle_origin[v] and i != vehicle_destination[v]))

    model.addConstrs((gp.quicksum(X[v, i, j] for v in range(V)) == 1 for i, j in operation_arc))
    M = 99999
    model.addConstrs((gp.quicksum(X[v, i, j] for i, j in operation_arc)) <= M * Y[v] for v in range(V))

    for t in time_stamp:
        model.addConstr(gp.quicksum(X[v, i, j] for v in range(V) for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j]) <= F)
        model.addConstr(gp.quicksum(X[v, i, j] for v in range(V) for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j]) <= F)

    model.addConstrs((Z[v, time_stamp[0]] == max_electric for v in range(V)))

    for v in range(V):
        for to, td in time_pair:
            consumption = gp.quicksum(
                X[v, i, j] * unit_consumption * (td - to) for i, j in operation_arc if
                node_time[i] <= to < td <= node_time[j])
            charging = gp.quicksum(X[v, i, j] * unit_charging * (td - to) for i, j in charging_arc if
                                   node_time[i] <= to< td <= node_time[j])
            model.addConstr(Z[v, td] == Z[v, to] - consumption + charging)

    model.addConstrs((Y[v] >= Y[v +1] for v in range(V - 1)), name='special1')

    model.addConstrs((X[v, 2 * T, T] == X[v, up_out_node, 2 * T + 1] for v in range(V)), name='special1')
    model.addConstrs((X[v, 2 * T, 0] == X[v, down_out_node, 2 * T + 1] for v in range(V)), name='special2')
    model.setParam('OutputFlag', 1)
    model.setParam('TimeLimit', 7200)
    model.setParam('Threads', 20)

    model.optimize()
    if model.SolCount > 0:
        vehicle = 0
        for v in range(V):
            if Y[v].x > 0.5:
                for i, j in vehicle_arc:
                    X_value[v, i, j] = X[v, i, j].x
                vehicle += 1
        running_time = time.time() - start_time

        return {'X_value': X_value, 'fleet_size': int(model.objVal), 'best_bound': int(model.ObjBound), 'running_time': running_time}
    else:
        return None
