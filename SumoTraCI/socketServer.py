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

from include.UtilityFunctions import SocketServerSimple
from include.UtilityFunctions import SumoVehicle
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

def TraciServer(server,dt):
    traci.start(["sumo-gui","-c", "Assets/Sumonity/SumoTraCI/sumoProject/opensource.sumocfg","--num-clients", "1", "-S"])
    junctionIDList = traci.junction.getIDList()
    traci.setOrder(0)

    step = 0
    while True:
        traci.simulationStep() # Trgger one timestep in the sumo simulation
        step += 1
        time.sleep(dt)

        simulationTime = traci.simulation.getTime()

        # do not get confused by the name vehicle list. this should be actor list ;)
        vehicleList = list()

        # Get common vehicles
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

        # Get persons/pedestrians
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
