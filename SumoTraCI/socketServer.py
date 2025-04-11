#!/usr/bin/env python

import argparse
from multiprocessing.connection import Client
import time
import threading
import os, sys
import traci
import traci.constants as tc
import json
import numpy as np
import math

from include.UtilityFunctions import SocketServerSimple
from include.UtilityFunctions import SumoVehicle
from include.UtilityFunctions import TrafficLight
from include.UtilityFunctions import Junction
from include.UtilityFunctions import SumoSimulationStepInfo
from include.path_calculator import find_point_ahead_on_path
from include.vehicle_manager import VehicleManager
from include.simulation_utils import is_valid_json, extract_number, clamp_value

debugWithoutUntiy = False



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
    else:
        # if we are inside a junction. we cannot update as the vehicle does not know its 
        # next lane in sumo. this is stupid, bus that's how sumo works here. The reasion 
        # is the getNextLinks Function.
        # path = globalPreviousPath
        path = globalPreviousPathDict[vehicle_id]


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

def calculate_point_ahead(current_pos, heading_degrees, distance, rotation_center=None):
    """
    Calculate point P2 that originates from P1 (current_pos)
    P1: current_pos (starting point)
    heading_degrees: the current heading of the object
    distance: distance to P2
    returns: tuple (x, y) of P2
    """
    if rotation_center is None:
        rotation_center = current_pos
        
    # Convert angle to radians
    angle_rad = math.radians(heading_degrees)
    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)
    
    # 1. Translate P1 to origin
    p1_translated = (
        current_pos[0] - rotation_center[0],
        current_pos[1] - rotation_center[1]
    )
    
    # 2. Rotate P1 to align with coordinate system
    p1_rotated = (
        p1_translated[0] * cos_theta - p1_translated[1] * sin_theta,
        p1_translated[0] * sin_theta + p1_translated[1] * cos_theta
    )
    
    # 3. Calculate P2 relative to rotated P1
    p2_local = (
        p1_rotated[0] + distance,  # Move forward by distance
        p1_rotated[1]
    )
    
    # 4. Rotate back
    p2_rotated = (
        p2_local[0] * cos_theta + p2_local[1] * sin_theta,
        -p2_local[0] * sin_theta + p2_local[1] * cos_theta
    )
    
    # 5. Translate back to world space
    x = p2_rotated[0] + rotation_center[0]
    y = p2_rotated[1] + rotation_center[1]
    
    return (x, y)

def TraciServer(server,dt):
    useWarmStart = False # beta feature, not fully implemented yet
    if useWarmStart:
        traci.start(["sumo-gui","-c", "Assets/Sumonity/SumoTraCI/sumoProject/opensource.sumocfg","--num-clients", "1", "--load-state", "Assets/Sumonity/SumoTraCI/sumoProject/warm_up/warm_up_state.xml", "-S"])
    else:
        traci.start(["sumo-gui","-c", "Assets/Sumonity/SumoTraCI/sumoProject/opensource.sumocfg","--num-clients", "1", "-S"])



        

    junctionIDList = traci.junction.getIDList()
    traci.setOrder(0)

    step = 0
    while True:
        start_time = time.time()
        traci.simulationStep() # Trgger one timestep in the sumo simulation
        step += 1
        time.sleep(dt)

        simulationTime = traci.simulation.getTime()


        # ==============================
        # Send Data from SUMO to Unity
        # ==============================

        # ----====Vehicles====----

        # do not get confused by the name vehicle list. this should be actor list ;)
        vehicleList = list()
        # Get common vehicles
        idList = traci.vehicle.getIDList()
        for i in range(0,len(idList)):
            id = idList[i]

            if id == "bike1":
                # lookaheadPos = (0,0)
                continue

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

            isInsideVehicle = False

            stop_state = 0
            veh = SumoVehicle(id,pos,rot,speed,signals,vehType,lookaheadPos,stop_state, isInsideVehicle)
            vehicleList.append(veh.__dict__)

        # ----====Persons====----
        # Get persons/pedestrians
        idList = traci.person.getIDList()
        for i in range(0,len(idList)):
            id = idList[i]
            pos = traci.person.getPosition(id)
            rot = traci.person.getAngle(id)
            rot = rot-90
            speed = traci.person.getSpeed(id)
            vehType = traci.person.getVehicleClass(id)
            signals = -1

            # not valid for car (todo implement pedestrian class)
            # Check if person is inside a vehicle
            isInsideVehicle = True
            if traci.person.getVehicle(id) == "": # returns vehicle id if person is inside a vehicle
                isInsideVehicle = False
                

            # Calculate lookahead point based on position and heading
            
            # Define lookahead distance
            lookahead_dist = 2.0  # 2 meters ahead
            
            # Calculate lookahead point using trigonometry
            lookaheadPos = calculate_point_ahead(pos, rot, lookahead_dist)

            stop_state = 0
            veh = SumoVehicle(id,pos,rot,speed,signals,vehType,lookaheadPos,stop_state,isInsideVehicle)
            vehicleList.append(veh.__dict__)


        # ----====TrafficLights====----
        # get tls
        junctionIDList = traci.trafficlight.getIDList()
        junctionList = list()
        for junctionID in junctionIDList:
            tlsStateAllStreams = traci.trafficlight.getRedYellowGreenState(junctionID)
            controlledLanes = traci.trafficlight.getControlledLanes(junctionID)
            tlsState = ''
            armPos = []
            last_lane = ''
            for i, lane in enumerate(controlledLanes): #Assumes simple signal plan, no priority left turning etc.
                if lane[:-2] != last_lane:
                    last_lane = lane[:-2]
                    try:
                        armPos.append(traci.lane.getShape(traci.trafficlight.getControlledLinks(junctionID)[i][0][2])[0])
                        tlsState += tlsStateAllStreams[i] # tlsState a string with with one Char for each arm
                    except:
                        traci.simulation.writeMessage('Cant find lane since pedestrian crossing')
            tls = TrafficLight(tlsState)
            jxn = Junction(junctionID,traci.junction.getPosition(junctionID),traci.trafficlight.getPhase(junctionID),tlsState,armPos)
            junctionList.append(jxn.__dict__)




        sumoSimStep = SumoSimulationStepInfo(simulationTime,vehicleList,junctionList).__dict__
        server.messageToSend = json.dumps(sumoSimStep)
       
        # ====================================
        # Receive Values from SUMO to Unity
        # ====================================
        try: 
            msg = json.loads(server.messageReceived)
            edgeID = ""
            laneVal = 1
            x = msg["positionX"]
            y = msg["positionY"]
            angleVal = msg["rotation"]+90
            keepRouteVal = 2
            matchThresholdVal = 100
            traci.vehicle.moveToXY(msg["id"], edgeID, laneVal, x, y, angleVal, keepRoute=keepRouteVal, matchThreshold=matchThresholdVal)
        except Exception as e:
            print("Error: ", str(e))
            pass

        execution_time = time.time() - start_time
        temp_dt = dt - execution_time
        step += 1

        if temp_dt < 0:
            print(f"Warning: Execution time exceeds the specified time step. temp_dt: {temp_dt}")
            temp_dt = 0

        print("Step:", step, "Time:", traci.simulation.getTime(), "Execution Time:", execution_time, "Temp DT:", temp_dt)
        time.sleep(temp_dt)


        execution_time = time.time() - start_time
        print("Total Execution Time:", execution_time)



    
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

def parse_arguments():
    parser = argparse.ArgumentParser(description='SUMO-Unity Bridge Server')
    parser.add_argument('--dt', 
                       type=float, 
                       required=True,
                       help='Simulation timestep in seconds')
    return parser.parse_args()

# ---=========---
#      MAIN
# ---=========---
if __name__ == '__main__':
    args = parse_arguments()
    # dt must be aligned with the unity simulation
    dt = args.dt
    
    server = SocketServerSimple("127.0.0.1", 25001, dt)
    server.messageToSend = "default"

    thread1 = threading.Thread(target=server.start)
    thread1.start()

    thread2 = threading.Thread(target=ServerStarted, args=(server,))
    thread2.start()
