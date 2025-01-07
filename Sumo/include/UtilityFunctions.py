# UtilityFunctions.py
import socket
import time
import traci


class SumoSimulationStepInfo:
    time = 0
    trafficLightList = list()
    vehicleList = list()
    message = "no message"
    # tlsList = list()  
    personalList = list()

    def __init__(self, _time, _vehicleList, _junctionList, _message, _personalList):
        self.time = _time
        self.vehicleList = _vehicleList
        self.junctionList = _junctionList
        self.message = _message
        self.personalList = _personalList

    def getVehicleInfo(self, id):
        for veh in self.vehicleList:
            if veh["id"]==id:
                return veh

class TrafficLight:
    state = ""
    def __init__(self, _state):
        self.state = _state

class Junction:
    id = 0
    pos = [0,0]
    phase = 0
    state = ""
    armPos = [[0,0]]

    def __init__(self,_id,_pos,_phase,_state,_armpos):
        self.id = _id
        self.pos = _pos
        self.phase = _phase
        self.state = _state
        self.armPos = _armpos

class SumoVehicle:
    id = ""
    positionX = 0
    positionY = 0
    rotation = 0
    speed = 0
    signals = None
    vehicleType = ""
    lookaheadPosX = 0
    lookaheadPosY = 0
    stopState = 0
    def __init__(self, _id, _pos, _rot, _speed, _signals, _vehType,_lookaheadPos, _stopState):
        self.id = _id
        self.positionX = _pos[0]
        self.positionY = _pos[1]
        self.rotation = _rot
        self.speed = _speed
        self.signals = _signals
        self.vehicleType = _vehType
        self.lookaheadPosX = _lookaheadPos[0]
        self.lookaheadPosY = _lookaheadPos[1]
        self.stopState = _stopState

class SocketServerSimple:
    HOST = "127.0.0.1"
    PORT = 25001

    messageToSend = ""
    messageReceived = ""
    messageSize = 1024

    delta = 0.5
    nrOfConnection = 0
    conn = None  # Initialized to None

    def __init__(self, ip=HOST, port=PORT, delta=0.015, nrListeners=1, messageSize=1024):
        self.PORT = port
        self.HOST = ip
        self.delta = delta
        self.nrListeners = nrListeners
        self.messageSize = messageSize

    def is_connected(self):
        return self.conn is not None

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.HOST,self.PORT))
        except OSError as e:
            print(f"OSError: {e}")

        sock.listen(self.nrListeners)
        conn, address = sock.accept()
        self.conn = conn
        print("Connection from: " + str(address))

        while True:
            # Send Data
            conn.sendall(self.messageToSend.encode("UTF-8"))

            # Receive Data
            receivedData = conn.recv(1024).decode("UTF-8")
            self.messageReceived = receivedData
            if receivedData is not None:
                pass
            time.sleep(self.delta)
        sock.close()

class TrafficLightController:
    def __init__(self, det_id_reg, det_id_dereg, tls_id, lane_number, lane_number_bus, bus_phase, route_id):
        self.det_id_reg = det_id_reg
        self.det_id_dereg = det_id_dereg
        self.tls_id = tls_id
        self.lane_number = lane_number
        self.lane_number_bus = lane_number_bus
        self.bus_phase = bus_phase
        self.route_id = route_id
        self.status = False #controller has not been triggered
        self.counter = 0
        #self.wasActivated = False

    def setStatus(self, boolStatus):
        self.status = boolStatus
        return self.status
    
    def getStatus(self):
        return self.status
    
    def switchStatus(self):
        return not self.status
    
    def bikeSwitch(self): #mabey use setLinkState or modolowie 1
        if traci.trafficlight.getRedYellowGreenState(self.tls_id)[self.lane_number] in ("g" , "G"):
            traci.trafficlight.setPhase(self.tls_id, traci.trafficlight.getPhase(self.tls_id)+1)
        elif traci.trafficlight.getRedYellowGreenState(self.tls_id)[self.lane_number] in ("y"):
            return
        elif traci.trafficlight.getRedYellowGreenState(self.tls_id)[self.lane_number_bus] in ("y"):
            traci.trafficlight.setPhase(self.tls_id, self.bus_phase)
        else:
            traci.trafficlight.setPhaseDuration(self.tls_id, 20)

    def busSwitch(self):
        traci.trafficlight.setPhaseDuration(self.tls_id, 1)
        self.setStatus(False)

    def getTLSId(self):
        return self.tls_id

    def addBus(self, key):
        if not self.status:
            busId  = key + "_" + str(self.counter)
            traci.vehicle.add(busId, 
                                self.route_id, 
                                typeID='DEFAULT_TAXITYPE',
                                depart='now', 
                                departLane='first', 
                                departPos='base', 
                                departSpeed='0', 
                                arrivalLane='current', 
                                arrivalPos='max', 
                                arrivalSpeed='current', 
                                fromTaz='', 
                                toTaz='', 
                                line='', 
                                personCapacity=0, 
                                personNumber=0)
            self.counter += 1
            self.setStatus(True)
