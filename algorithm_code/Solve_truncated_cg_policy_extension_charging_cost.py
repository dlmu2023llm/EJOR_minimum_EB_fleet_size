from gurobipy import GRB
import gurobipy as gp
from Parameter import *
import time
from Intial_solution_extension_charging_cost import *
import copy

def truncated_column_generation(para):
    overall_start_time = time.time()
    unfinished_task = [(i, j) for i, j in para['operation_arc']]
    charging_count = {t: {0: [], 1: []} for t in range(para['T'])}
    total_fleet_size = 0
    finished_task = []
    vehicle_arc = para['vehicle_arc']

    V = para['V']
    operation_arc = para['operation_arc']
    vehicle_task_volume = {v: 0 for v in range(V)}
    X_value = {(v, i, j): 0 for v in range(V) for i, j in vehicle_arc}
    while len(unfinished_task) > 0:
        now_unfinished_task = len(unfinished_task)
        unfinished_task, current_fleet, now_finished_task, X_value_current = column_generation(para, unfinished_task, charging_count)
        finished_task += [(i, j) for i, j in now_finished_task]
        for fleet in range(current_fleet):
            for i, j in vehicle_arc:
                if (fleet, i, j) not in X_value_current:
                    X_value[total_fleet_size + fleet, i, j] = 0
                elif X_value_current[fleet, i, j] > 0.5:
                    X_value[total_fleet_size + fleet, i, j] = 1
                    if (i, j) in operation_arc:
                        vehicle_task_volume[total_fleet_size + fleet] += 1
                elif X_value_current[fleet, i, j] < 0.5:
                    X_value[total_fleet_size + fleet, i, j] = 0
            now_unfinished_task -= vehicle_task_volume[total_fleet_size + fleet]

        total_fleet_size += current_fleet

    return {'X_value': X_value, 'fleet_size': total_fleet_size, 'running_time': time.time() - overall_start_time}

def column_generation(para, unfinished_task, charging_count):
    Z_min = 0.005
    I = 100
    threshold = 0.7
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
    operation_arc = [(i, j) for i, j in unfinished_task]
    waiting_arc = para['waiting_arc']
    charging_arc = para['charging_arc']
    charging_arc_cost = para['charging_arc_cost']
    charging_arc_p0 = para['charging_arc_p0']
    charging_arc_p1 = para['charging_arc_p1']
    poll_arc = para['poll_arc']
    time_stamp = para['time_stamp']
    up_out_node = para['up_out_node']
    down_out_node = para['down_out_node']
    node_time = {n: n if n < T else n - T for n in range(2 * T)}
    time_pair = [(time_stamp[t], time_stamp[t + 1]) for t in range(len(time_stamp) - 1)]
    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    node_out_arc = {i: [] for i in total_node}
    node_in_arc = {i: [] for i in total_node}
    for i, j in vehicle_arc:
        node_out_arc[i].append((i, j))
        node_in_arc[j].append((i, j))

    start_time = time.time()

    unfinished_task_greedy = [(i, j) for i, j in unfinished_task]
    occupation_charger_greedy = copy.deepcopy(charging_count)
    result = greedy_initial_policy(para, unfinished_task_greedy, occupation_charger_greedy)
    X_value_greedy = result['X_value']
    B = result['fleet_size']
    xi = []
    for b in range(B):
        xi.append({})
        for i, j in vehicle_arc:
            if X_value_greedy[b, i, j] >= 0.5:
                xi[b][i, j] = 1
            else:
                xi[b][i, j] = 0

    shadow_price_0 = {(i, j): 0 for i, j in operation_arc}
    shadow_price_1 = {t: 0 for t in time_stamp}
    shadow_price_2 = {t: 0 for t in time_stamp}
    RP_obj_list = []
    q = 1
    current_finished_task = []
    X_value_current = {}
    current_column = 0
    while True:
        relax_model = gp.Model("RP")
        Y = relax_model.addVars(B, vtype=GRB.CONTINUOUS, name="Y", lb=0)
        obj = gp.quicksum(Y[b] for b in range(B))
        obj1 = gp.quicksum(charging_arc_cost[i,j] * xi[b][i, j] * Y[b] for b in range(B) for i, j in charging_arc)
        relax_model.setObjective(obj + obj1, GRB.MINIMIZE)
        relax_model.addConstrs((gp.quicksum(xi[b][i, j] * Y[b] for b in range(B)) == 1 for i, j in operation_arc),
                               name="con0")
        relax_model.addConstrs((gp.quicksum(xi[b][i, j] * Y[b] for b in range(B) for (i, j) in charging_arc_p0 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][0]) for t in time_stamp), name='con1')
        relax_model.addConstrs((gp.quicksum(xi[b][i, j] * Y[b] for b in range(B) for (i, j) in charging_arc_p1 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][1]) for t in time_stamp), name='con2')
        relax_model.setParam('OutputFlag', 0)
        relax_model.setParam('Threads', 20)
        relax_model.optimize()

        RP_obj_list.append(relax_model.ObjVal)
        if q >= I and (RP_obj_list[q - 2] - RP_obj_list[q - 1]) / RP_obj_list[q - 2] <= Z_min:
            break
        previous_shadow_price_0 = {(i, j): shadow_price_0[i, j] for i, j in operation_arc}
        previous_shadow_price_1 = {t: shadow_price_1[t] for t in time_stamp}
        previous_shadow_price_2 = {t: shadow_price_2[t] for t in time_stamp}
        shadow_price_0 = {(i, j): relax_model.getConstrByName('con0[{},{}]'.format(i, j)).Pi for i, j in operation_arc}
        shadow_price_1 = {t: relax_model.getConstrByName('con1[{}]'.format(t)).Pi for t in time_stamp}
        shadow_price_2 = {t: relax_model.getConstrByName('con2[{}]'.format(t)).Pi for t in time_stamp}
        if all(abs(shadow_price_0[i, j] - previous_shadow_price_0[i, j]) < 1e-6 for i, j in operation_arc) and all(abs(shadow_price_1[t] - previous_shadow_price_1[t]) < 1e-6 for t in time_stamp) and all(abs(shadow_price_2[t] - previous_shadow_price_2[t]) < 1e-6 for t in time_stamp):
            break
        sub_model = gp.Model("SUB")
        X = sub_model.addVars(vehicle_arc, vtype=GRB.BINARY, name='X')
        Z = sub_model.addVars(time_stamp, lb=min_electric, ub=max_electric, vtype=GRB.CONTINUOUS, name='Z')

        obj = 1 - gp.quicksum(shadow_price_0[i, j] * X[i, j] for i, j in operation_arc) - gp.quicksum(shadow_price_1[t] * X[i, j] for t in time_stamp for i, j in charging_arc_p0 if node_time[i] <= t < node_time[j]) - gp.quicksum(shadow_price_2[t] * X[i, j] for t in time_stamp for i, j in charging_arc_p1 if node_time[i] <= t < node_time[j])
        sub_model.setObjective(obj, GRB.MINIMIZE)

        sub_model.addConstr(gp.quicksum(X[m, n] for m, n in node_out_arc[vehicle_origin[0]]) - gp.quicksum(
            X[m, n] for m, n in node_in_arc[vehicle_origin[0]]) == 1, name='origin_flow')
        sub_model.addConstr(gp.quicksum(X[m, n] for m, n in node_out_arc[vehicle_destination[0]]) - gp.quicksum(
            X[m, n] for m, n in node_in_arc[vehicle_destination[0]]) == -1, name='destination_flow')
        sub_model.addConstrs(
            (gp.quicksum(X[m, n] for m, n in node_out_arc[i]) - gp.quicksum(X[m, n] for m, n in node_in_arc[i]) == 0
             for i in total_node if i != vehicle_origin[0] and i != vehicle_destination[0]), name='inter_flow')

        sub_model.addConstr(Z[time_stamp[0]] == max_electric, name='e_start')

        for to, td in time_pair:
            consumption = gp.quicksum(
                X[i, j] * unit_consumption * (td - to) for i, j in operation_arc
                if node_time[i] <= to < td <= node_time[j])
            charging = gp.quicksum(
                X[i, j] * unit_charging * (td - to) for i, j in charging_arc if node_time[i] <= to < td <= node_time[j])
            sub_model.addConstr(Z[td] == Z[to] - consumption + charging, name='transfer({},{})'.format(to, td))
        sub_model.addConstr(X[2 * T, T] == X[up_out_node, 2 * T + 1], name='special1')
        sub_model.addConstr(X[2 * T, 0] == X[down_out_node, 2 * T + 1], name='special2')
        sub_model.setParam('OutputFlag', 0)
        sub_model.setParam('Threads', 20)

        sub_model.optimize()
        if sub_model.status == GRB.OPTIMAL:
            if sub_model.objVal > 0:
                break
            xi.append({})
            for i, j in vehicle_arc:
                xi[B][i, j] = 1 if X[i, j].x >= 0.5 else 0
            B += 1
        else:
            print('No solution found, exiting.')
            break
        q += 1

    relax_model = gp.Model("Binary_relax")
    Y = relax_model.addVars(B, vtype=GRB.CONTINUOUS, name="Y", lb=0)
    obj = gp.quicksum(Y[b] for b in range(B))
    obj1 = gp.quicksum(charging_arc_cost[i, j] * xi[b][i, j] * Y[b] for b in range(B) for i, j in charging_arc)
    relax_model.setObjective(obj + obj1, GRB.MINIMIZE)
    relax_model.addConstrs((gp.quicksum(xi[b][i, j] * Y[b] for b in range(B)) == 1 for i, j in operation_arc),
                           name="con")
    relax_model.addConstrs((gp.quicksum(
        xi[b][i, j] * Y[b] for b in range(B) for (i, j) in charging_arc_p0 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][0]) for
                            t in time_stamp), name='con1')
    relax_model.addConstrs((gp.quicksum(
        xi[b][i, j] * Y[b] for b in range(B) for (i, j) in charging_arc_p1 if node_time[i] <= t < node_time[j]) <= F - len(charging_count[t][1]) for
                            t in time_stamp), name='con2')
    relax_model.setParam('OutputFlag', 0)
    relax_model.setParam('Threads', 20)
    relax_model.optimize()

    current_fleet = 0
    max_b_index = 0
    max_b_value = 0
    column_task = []
    for b in range(B):
        b_task_num = 0
        for i, j in operation_arc:
            if xi[b][i, j] >= 0.5:
                b_task_num += 1
        column_task.append(b_task_num)
    threshold_flag = False
    chosen_B = []
    for b in range(B):
        if Y[b].x > threshold:
            threshold_flag = True
            chosen_B.append(b)
        if Y[b].x > max_b_value or Y[b].x == max_b_value and column_task[b] > column_task[max_b_index]:
            max_b_index = b
            max_b_value = Y[b].x
    if threshold_flag:
        for b in chosen_B:
            for i, j in vehicle_arc:
                if xi[b][i, j] > 0.5:
                    X_value_current[current_column, i, j] = 1
                else:
                    X_value_current[current_column, i, j] = 0
            current_column += 1
            current_task = 0
            for i, j in operation_arc:
                if xi[b][i, j] > 0.5:
                    unfinished_task.remove((i, j))
                    current_finished_task.append((i, j))
                    current_task += 1

            for i,j in charging_arc_p0:
                if xi[b][i, j] >= 0.5:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][0].append(b)
            for i,j in charging_arc_p1:
                if xi[b][i, j] >= 0.5:
                    for t in range(node_time[i], node_time[j]):
                        charging_count[t][1].append(b)

        current_fleet += len(chosen_B)
    else:
        for i, j in vehicle_arc:
            if xi[max_b_index][i, j] > 0.5:
                X_value_current[current_column, i, j] = 1
            else:
                X_value_current[current_column, i, j] = 0
        current_column += 1

        current_task = 0
        for i, j in operation_arc:
            if xi[max_b_index][i, j] > 0.5:
                unfinished_task.remove((i, j))
                current_finished_task.append((i, j))
                current_task += 1

        for i, j in charging_arc_p0:
            if xi[max_b_index][i, j] >= 0.5:
                for t in range(node_time[i], node_time[j]):
                    charging_count[t][0].append(max_b_index)
        for i, j in charging_arc_p1:
            if xi[max_b_index][i, j] >= 0.5:
                for t in range(node_time[i], node_time[j]):
                    charging_count[t][1].append(max_b_index)
        current_fleet += 1

    return unfinished_task, current_fleet, current_finished_task, X_value_current

