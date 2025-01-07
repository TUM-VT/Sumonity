#!/usr/bin/env python

from multiprocessing.connection import Client
from pickle import TRUE
import time
import threading
import random
import os, sys
from turtle import width
import traci
import traci.constants as tc
import json
import numpy as np
import math
from typing import List, Tuple
import re

import traceback # for debugging exceptions

from include.UtilityFunctions import SocketServerSimple
from include.UtilityFunctions import SumoVehicle
from include.UtilityFunctions import SumoSimulationStepInfo

debugWithoutUntiy = False

def is_valid_json(s):
    try:
        json.loads(s)
        return True
    except ValueError:
        return False
##--------------------------------
def calculate_distance(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def closest_points(point_list: List[List[Tuple[float, float]]], dynamic_point: Tuple[float, float]) -> List[Tuple[float, float]]:
    # Flatten the list of points
    flattened_points = [point for sublist in point_list for point in sublist]

    # Calculate distances from the dynamic point
    distances = [np.linalg.norm(np.array(point) - np.array(dynamic_point)) for point in flattened_points]

    # Find the two closest points
    closest_indices = np.argsort(distances)[:2]

    # Sort the indices to maintain order from the original list
    closest_indices.sort()

    # Get the closest points in the original order
    closest_points = [flattened_points[i] for i in closest_indices]

    return closest_points

def remove_duplicates(point_list: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
    unique_points = set()
    new_list = []

    for sublist in point_list:
        new_sublist = []
        for point in sublist:
            if point not in unique_points:
                unique_points.add(point)
                new_sublist.append(point)
        if new_sublist:
            new_list.append(new_sublist)

    return new_list

def remove_before_point(point_list: List[List[Tuple[float, float]]], search_point: Tuple[float, float]) -> List[List[Tuple[float, float]]]:
    found = False
    new_list = []

    for point in point_list:
        if found:
            # If the point has been found, add all subsequent sublists
            new_list.append(point)
        else:
            if point == search_point:
                found = True
                new_list.append(point)



    return new_list


def flatten_and_remove_duplicates(point_list: List[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
    # Flatten the list of points and remove duplicates
    unique_points = set()
    flattened_points = []

    for sublist in point_list:
        for point in sublist:
            if point not in unique_points:
                unique_points.add(point)
                flattened_points.append(point)

    return flattened_points

def closest_segment_point_ahead(points, position):
    min_distance = float('inf')
    # closest_segment = None
    point_ahead = None

    for i in range(len(points) - 1):
        p1, p2 = np.array(points[i]), np.array(points[i + 1])
        p = np.array(position)

        # Compute the projection of point p onto the line defined by p1 and p2
        line_vec = p2 - p1
        point_vec = p - p1
        line_len = np.linalg.norm(line_vec)
        line_unitvec = line_vec / line_len
        point_vec_scaled = point_vec / line_len
        t = np.dot(line_unitvec, point_vec_scaled)    

        # Ensure t is within the bounds of the line segment
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0

        nearest = line_vec * t
        dist = np.linalg.norm(point_vec - nearest)

        if dist < min_distance:
            min_distance = dist
            # closest_segment = [p1, p2]
            point_ahead = (p2[0],p2[1])

    return point_ahead


def find_point_at_distance(point_list, distance):
    def calculate_distance(p1, p2):
        return np.linalg.norm(np.array(p2) - np.array(p1))
    
    running_total = 0
    prev_point = point_list[0]

    for point in point_list[1:]:
        segment_length = calculate_distance(prev_point, point)
        
        if running_total + segment_length >= distance:
            # Calculate the exact point on this segment
            excess_distance = distance - running_total
            direction_vector = np.array(point) - np.array(prev_point)
            unit_vector = direction_vector / np.linalg.norm(direction_vector)
            exact_point = np.array(prev_point) + unit_vector * excess_distance
            return tuple(exact_point)

        running_total += segment_length
        prev_point = point

    # If the distance exceeds the length of the path, return the last point
    return point_list[-1]

def find_point_ahead_on_path(path, current_position, distance_ahead):
    """Find a point on the path that is a certain distance ahead of the current position."""

    # path = remove_duplicates(path)

    # i have to find the two points that are closest to me
    # points = closest_points(path, current_position)
    
    # the segment we are driving on
    path = flatten_and_remove_duplicates(path)
    segmentPointAhead = closest_segment_point_ahead(path, current_position)


    updatedPath = remove_before_point (path, segmentPointAhead)

    # # make the path 1D
    # updatedPath = flatten_and_remove_duplicates(updatedPath)

    # add ego to updated path
    updatedPath.insert(0, current_position)

    # Debug path

    # Use the combined function with the provided list, dynamic point, and max distance
    # Find the point at the specified distance
    point_at_specific_distance = find_point_at_distance(updatedPath, distance_ahead)


    return(point_at_specific_distance)

def extract_number(input_string):
    # Regular expression pattern to match a number between ':' and '_'
    pattern = r":(\d+)_"

    # Find all matches in the input string
    match = re.search(pattern, input_string)

    # If a match is found, return the number
    if match:
        return match.group(1)  # group(1) returns the first captured group
    else:
        return None

# uncool coding style ;) 
globalPreviousPathDict = {}

def predict_future_position(vehicle_id, minLookaheadDistance, maxLookaheadDistance, pos, junctionIDList, speed):
    global globalPreviousPathDict
    current_lane = traci.vehicle.getLaneID(vehicle_id)


    nextLinks = traci.vehicle.getNextLinks(vehicle_id)

    LaneID = extract_number(current_lane)

    isJunction = None
    if not(LaneID==None):
        isJunction = any(LaneID in s for s in junctionIDList)


    if isJunction==None:
        # The current lane may change as a result of lane changes that can occur locally
        lanebasedRoute = []
        lanebasedRoute.append(current_lane)

        for inner_tuple in nextLinks:
            lanebasedRoute.append(inner_tuple[4])
            lanebasedRoute.append(inner_tuple[0])


        routeShape = []
        for lanes in lanebasedRoute:
            routeShape.append(traci.lane.getShape(lanes))

        path = routeShape
        globalPreviousPathDict[vehicle_id] = path
        # globalPreviousPath = path
    else:
        # if we are inside a junction. we cannot update as the vehicle does not know its 
        # next lane in sumo. this is stupid, bus that's how sumo works here. The reasion 
        # is the getNextLinks Function.
        # path = globalPreviousPath
        path = globalPreviousPathDict[vehicle_id]


    # current_position = (294, 468)  # Replace with the vehicle's current position
    current_position = (pos[0],pos[1])  # Replace with the vehicle's current position

    # speed dependent lookahead value, to increase control stability
    if (speed/3.6)<minLookaheadDistance:
        lookaheadDistance = minLookaheadDistance
    else:
        lookaheadGain = 1
        lookaheadDistance = (speed/3.6)*lookaheadGain

    lookaheadDistance = clamp_value(lookaheadDistance,minLookaheadDistance,maxLookaheadDistance)

    try:
        point_ahead = find_point_ahead_on_path(path, current_position, lookaheadDistance)
        return(point_ahead)
    except ValueError as e:
        print(e)
        return (0)
    

def clamp_value(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def TraciServer(server,dt):


    traci.start(["sumo-gui","-c", "Assets/Sumonity/SumoTraCI/sumoProject/test_opensource_soma.sumocfg","--num-clients", "1", "-S"])

    # get list of junctions:
    junctionIDList = traci.junction.getIDList()

    traci.setOrder(0)



    step = 0
    while True:
        traci.simulationStep() # Trgger one timestep in the sumo simulation
        step += 1
        time.sleep(dt)


        # ==============================
        # Send Values from SUMO to Unity
        # ==============================
        simulationTime = traci.simulation.getTime()

        # do not get confused by the name vehicle list. this should be actor list ;)
        vehicleList = list()

        # get common vehicles
        idList = traci.vehicle.getIDList()
        for i in range(0,len(idList)):
            id = idList[i]
            pos = traci.vehicle.getPosition(id)
            rot = traci.vehicle.getAngle(id)
            rot = rot+180
            speed = traci.vehicle.getSpeed(id)
            signals = traci.vehicle.getSignals(id)
            vehType = traci.vehicle.getVehicleClass(id)

            # get lookahaed point
            minLookaheadDistance = 7
            maxLookaheadDistance = 10
            lookaheadPos = predict_future_position(id, minLookaheadDistance, maxLookaheadDistance, pos, junctionIDList, speed)  
            # Debug target point
            # print("Point 10 meters ahead:", lookaheadPos)


            veh = SumoVehicle(id,pos,rot,speed,signals,vehType,lookaheadPos)
            vehicleList.append(veh.__dict__)

        # get persons/pedestrians
        idList = traci.person.getIDList()
        for i in range(0,len(idList)):
            id = idList[i]
            pos = traci.person.getPosition(id)
            rot = traci.person.getAngle(id)
            rot = rot+180
            speed = traci.person.getSpeed(id)
            vehType = traci.person.getVehicleClass(id)
            signals = -1

            # get lookahaed point
            lookaheadPos = (0,0)

            veh = SumoVehicle(id,pos,rot,speed,signals,vehType,lookaheadPos)
            vehicleList.append(veh.__dict__)

        trafficLightPhase = 1

        sumoSimStep = SumoSimulationStepInfo(simulationTime,vehicleList,trafficLightPhase).__dict__
        server.messageToSend = json.dumps(sumoSimStep)



       
    traci.close()



def ServerStarted(server):
    if(debugWithoutUntiy):
        pass
    else:
        print("Waiting for Unity...")
        while not server.is_connected():
            time.sleep(0.1)
    thread2 = threading.Thread(target=TraciServer, args=(server, dt))
    thread2.start()



# ---=========---
#      MAIN
# ---=========---
if __name__ == '__main__':

    dt = 0.05
    server = SocketServerSimple("127.0.0.1", 25001, dt)
    server.messageToSend = "default"

    thread1 = threading.Thread(target=server.start)
    thread1.start()

    thread2 = threading.Thread(target=ServerStarted, args=(server,))
    thread2.start()
