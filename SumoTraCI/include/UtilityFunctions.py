# UtilityFunctions.py
import socket
import time
import traci


class SumoSimulationStepInfo:
    time = 0
    trafficLightList = list()
    vehicleList = list()
    # message = "no message"
    tlsList = list()  
    # personalList = list()

    def __init__(self, _time, _vehicleList, _junctionList):
        self.time = _time
        self.vehicleList = _vehicleList
        self.junctionList = _junctionList

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
    isInsideVehicle = False
    
    def __init__(self, _id, _pos, _rot, _speed, _signals, _vehType,_lookaheadPos, _stopState, _isInsideVehicle):
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
        self.isInsideVehicle = _isInsideVehicle

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

