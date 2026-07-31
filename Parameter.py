import numpy as np
import pickle

def get_underlying_para(t):
    T = t
    V = 25
    F = 8
    running_time = 35
    charging_time = 30
    unit_consumption = 0.7 / 2 / running_time
    unit_charging = 0.8 / charging_time
    max_electric = 0.9
    min_electric = 0.1
    parameter = {
        'T': T,
        'V': V,
        'F': F,
        'running_time': running_time,
        'charging_time': charging_time,
        'unit_consumption': unit_consumption,
        'unit_charging': unit_charging,
        'max_electric': max_electric,
        'min_electric': min_electric,
    }
    return parameter

def create_random_timetable(t):
    operation_arc = []
    para = get_underlying_para(t)
    T = para['T']
    running_time = para['running_time']
    up_current_node = T
    down_current_node = 0
    down_task = []
    up_task = []
    while down_current_node < T - running_time:
        down_task.append((down_current_node, down_current_node + T + running_time))
        down_current_node += np.random.randint(7, 12)  # high: (5, 10), medium: (7, 12), low: (9, 14)
    while up_current_node - T < T - running_time:
        up_task.append((up_current_node, up_current_node - T + running_time))
        up_current_node += np.random.randint(7, 12) # high: (5, 10), medium: (7, 12), low: (9, 14)
    min_task = min(len(down_task), len(up_task))
    for i in range(min_task):
        operation_arc.append(down_task[i])
        operation_arc.append(up_task[i])

    return operation_arc

def create_network(t):
    para = get_underlying_para(t)
    T = para['T']
    V = para['V']
    running_time = para['running_time']
    charging_time = para['charging_time']
    vehicle_origin = {v: 2 * T for v in range(V)}
    vehicle_destination = {v: 2 * T + 1 for v in range(V)}
    operation_arc = create_random_timetable(T)

    total_node = []

    for i, j in operation_arc:
        if i not in total_node:
            total_node.append(i)
        if j not in total_node:
            total_node.append(j)
    time_stamp = sorted(set(i % T for i in total_node))

    total_node.append(2 * T)
    total_node.append(2 * T + 1)
    total_node.sort()
    node_out_arc = {i: [] for i in total_node}
    node_in_arc = {i: [] for i in total_node}
    up_in_node = 10000
    down_in_node = 10000
    for i in total_node:
        if i < down_in_node and i < T:
            down_in_node = i
        if T <= i < 2 * T and i < up_in_node:
            up_in_node = i
    up_out_node = 0
    down_out_node = 0
    for i in total_node:
        if down_out_node < i < T:
            down_out_node = i
        if T <= i < 2 * T and i > up_out_node:
            up_out_node = i
    poll_arc = [(2 * T, 0), (2 * T, T), (up_out_node, 2 * T + 1), (down_out_node, 2 * T + 1)]

    waiting_arc = []
    for index in range(len(total_node) - 1):
        if total_node[index + 1] < T or total_node[index] >= T:
            if total_node[index + 1] != 2 * T:
                waiting_arc.append((total_node[index], total_node[index + 1]))

    charging_arc = []
    min_charging_time = 10
    for to in total_node:
        for td in total_node:
            if running_time <= to <= td < T - running_time or T + running_time <= to <= td <= 2 * T - running_time:
                if min_charging_time <= td - to <= charging_time:
                    if (to, td) not in waiting_arc:
                        charging_arc.append((to, td))
    charging_arc_cost = {}
    for (to, td) in charging_arc:
        charging_arc_cost[(to, td)] = 0.25 * (td - to)

    charging_arc_p0 = []
    charging_arc_p1 = []
    for (to, td) in charging_arc:
        if to < T and td < T:
            charging_arc_p0.append((to, td))
        else:
            charging_arc_p1.append((to, td))
    charging_arc_p0_cost = {}
    charging_arc_p1_cost = {}
    for (to, td) in charging_arc_p0:
        charging_arc_p0_cost[(to, td)] = 0.25 * (td - to)
    for (to, td) in charging_arc_p1:
        charging_arc_p1_cost[(to, td)] = 0.25 * (td - to)

    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    for i, j in waiting_arc + operation_arc + charging_arc + poll_arc:
        node_out_arc[i].append((i, j))
        node_in_arc[j].append((i, j))
    parameter = {
        'node_out_arc': node_out_arc,
        'node_in_arc': node_in_arc,
        'operation_arc': operation_arc,
        'waiting_arc': waiting_arc,
        'poll_arc': poll_arc,
        'charging_arc': charging_arc,
        'charging_arc_cost': charging_arc_cost,
        'charging_arc_p0': charging_arc_p0,
        'charging_arc_p1': charging_arc_p1,
        'charging_arc_p0_cost': charging_arc_p0_cost,
        'charging_arc_p1_cost': charging_arc_p1_cost,
        'vehicle_arc': vehicle_arc,
        'total_node': total_node,
        'vehicle_origin': vehicle_origin,
        'vehicle_destination': vehicle_destination,
        'down_in_node': down_in_node,
        'up_in_node': up_in_node,
        'down_out_node': down_out_node,
        'up_out_node': up_out_node,
        'time_stamp': time_stamp,
        **para
    }
    return parameter
