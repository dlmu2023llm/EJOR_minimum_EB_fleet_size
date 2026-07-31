import time
import numpy as np
from Parameter import *

def FIFO_policy(para):
    print('----FIFO----')
    T = para['T']
    V = para['V']
    F = para['F']
    unit_consumption = para['unit_consumption']
    unit_charging = para['unit_charging']
    max_electric = para['max_electric']
    min_electric = para['min_electric']
    running_time = para['running_time']
    operation_arc = para['operation_arc']
    waiting_arc = para['waiting_arc']
    charging_arc = para['charging_arc']
    poll_arc = para['poll_arc']
    vehicle_task_volume = {v: 0 for v in range(V)}
    vehicle_arc = waiting_arc + operation_arc + charging_arc + poll_arc
    node_time = {n: n if n < T else n - T for n in range(2 * T)}
    time_stamp = list(sorted(set(node_time.values())))
    total_running_time = 0
    start = time.time()

    X_value = {(v, i, j): 0 for v in range(V) for i, j in vehicle_arc}
    Z_value = {(v, t): 0 for v in range(V) for t in time_stamp}
    unfinished_task = [(i, j) for i, j in operation_arc]
    min_SOC = min_electric + unit_consumption * running_time

    station_poll = {0: [], 1: []}
    charging_count = {t: {0: [], 1: []} for t in range(T)}
    t = 0
    vehicle_fleet = 0
    while t < T:
        for v in range(vehicle_fleet):
            consumption = 0
            charging = 0
            for i, j in operation_arc:
                if X_value[v, i, j] == 1 and node_time[i] <= t - 1 < node_time[j]:
                    consumption = unit_consumption
            for i, j in charging_arc:
                if j < 2 * T and X_value[v, i, j] == 1 and node_time[i] <= t - 1 < node_time[j]:
                    charging = unit_charging
            Z_value[v, t] = Z_value[v, t - 1] + charging - consumption

        for v in range(vehicle_fleet):
            for i, j in operation_arc + charging_arc:
                if j < 2 * T and node_time[j] == t and X_value[v, i, j] == 1:
                    current_station = int(j / T)
                    station_poll[current_station].append(v)

        for i in range(2):
            for v in station_poll[i]:
                if Z_value[v, t] < min_SOC: 
                    if len(charging_count[t][i]) < F:
                        duration_time = int((max_electric - Z_value[v, t]) / unit_charging)
                        if (t + i * T, t + duration_time + i * T) not in charging_arc:
                            charging_arc.append((t + i * T, t + duration_time + i * T))
                        for vehicle in range(V):
                            if (vehicle, t + i * T, t + duration_time + i * T) not in X_value.keys():
                                X_value[vehicle, t + i * T, t + duration_time + i * T] = 0
                        X_value[v, t + i * T, t + duration_time + i * T] = 1
                        station_poll[i].remove(v)
                        if t + duration_time <= T:
                            for to in range(t, t + duration_time):
                                charging_count[to][i].append(v)
                        else:
                            for to in range(t, T):
                                charging_count[to][i].append(v)
                    else:
                        continue

        unfinished_task_copy = [(i, j) for i, j in unfinished_task]
        for i, j in unfinished_task_copy:
            if node_time[i] == t:
                current_station = int(i / T)
                if len(station_poll[current_station]) == 0:
                    station_poll[current_station].append(vehicle_fleet)
                    Z_value[vehicle_fleet, t] = max_electric
                    current_vehicle = station_poll[current_station][-1]
                    vehicle_task_volume[current_vehicle] += 1
                    X_value[current_vehicle, i, j] = 1
                    unfinished_task.remove((i, j))
                    station_poll[current_station].pop(-1)
                    vehicle_fleet += 1
                else:
                    assigned = False
                    for idx, sta_vehicle in enumerate(station_poll[current_station]):
                        if Z_value[sta_vehicle, t] >= min_SOC:
                            current_vehicle = sta_vehicle
                            vehicle_task_volume[current_vehicle] += 1
                            X_value[current_vehicle, i, j] = 1
                            unfinished_task.remove((i, j))
                            station_poll[current_station].pop(idx)
                            assigned = True
                            break

                    if not assigned:
                        station_poll[current_station].append(vehicle_fleet)
                        Z_value[vehicle_fleet, t] = max_electric
                        current_vehicle = station_poll[current_station][-1]
                        vehicle_task_volume[current_vehicle] += 1
                        X_value[current_vehicle, i, j] = 1
                        unfinished_task.remove((i, j))
                        station_poll[current_station].pop(-1)
                        vehicle_fleet += 1

        t += 1
        if len(unfinished_task) == 0:
            total_running_time = time.time() - start
            break
            
    return {'X_value': X_value, 'fleet_size': vehicle_fleet, 'charging_arc': charging_arc,
            'running_time': total_running_time}
