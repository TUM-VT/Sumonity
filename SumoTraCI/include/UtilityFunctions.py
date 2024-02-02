# UtilityFunctions.py
import socket
import time


class SumoSimulationStepInfo:
    time = 0
    trafficLightPhase = 0
    vehicleList = list()      
    #personList = list()

    def __init__(self, _time, _vehicleList, _trafficLightPhase, _personList=list()):
        self.time = _time
        self.vehicleList = _vehicleList
        self.trafficLightPhase = _trafficLightPhase
        #self.personList = _personList

    def getVehicleInfo(self, id):
        for veh in self.vehicleList:
            if veh["id"]==id:
                return veh


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
    def __init__(self, _id, _pos, _rot, _speed, _signals, _vehType,_lookaheadPos):
        self.id = _id
        self.positionX = _pos[0]
        self.positionY = _pos[1]
        self.rotation = _rot
        self.speed = _speed
        self.signals = _signals
        self.vehicleType = _vehType
        self.lookaheadPosX = _lookaheadPos[0]
        self.lookaheadPosY = _lookaheadPos[1]

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