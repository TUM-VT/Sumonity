using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Threading;

namespace tumvt.sumounity
{
    public interface IVehicleController
    {
        string id { get; set; }
    }

    public class SumoSocketClient : MonoBehaviour
    {

        [System.Serializable]
        private class DebugData 
        {
            [Tooltip("Message received from/to SUMO")]
            public string messageReceived;
            public string messageToSend;
            public bool showDebugGizmoSimulatorEgo = false;
        }
        [Header("Socket Communication")]
        [SerializeField] private DebugData debugData;


        [System.Serializable]
        private class SimulatorVehicleInfo
        {
            [Tooltip("The ID of the ego vehicle in SUMO used for inserting vehicles into the SUMO simulation")]
            public string egoVehicleId = "bike1";
            [Tooltip("The Transform of the ego vehicle in Unity used for inserting vehicles into the SUMO simulation")]
            public Transform egoVehicle;
            [Tooltip("Send data to SUMO")]
            public bool _sendData;
        }
        [Header("Simulator Vehicle Configuration")]
        [SerializeField] private SimulatorVehicleInfo simulatorVehicleInfo;
        public bool SendData => simulatorVehicleInfo._sendData;
        public Transform EgoVehicle => simulatorVehicleInfo.egoVehicle;



        [System.Serializable]
        private class OptimizationSettings
        {
            [Tooltip("If enabled SUMO-Simulation runs in background and no vehicles are spawned")]
            public bool RunSumoInBackground = false;
            public bool useEgoRadius = false;
            [Range(0, 500)] 
            public float egoRadius = 100;
        }
        
        [System.Serializable]
        private class VehicleToggles
        {
            public bool busEnable = false;
            public bool pedestrianEnable = false;
        }

        [Header("Settings")]
        [Space(10)]
        [SerializeField] private OptimizationSettings optimizationSettings;
        [SerializeField] private VehicleToggles vehicleToggles;

        [Header("Simulation Step Information")]
        [SerializeField] private SumoSimulationStepInfo _stepInfo;
        public SumoSimulationStepInfo StepInfo => _stepInfo;

        [System.Serializable]
        private class VehicleSetup
        {
            public VehicleCompositionsScriptableObject vehicleCompositions;
            public Dictionary<string, GameObject> vehDict = new Dictionary<string, GameObject>();
        }

        [Header("SUMO Vehicles")]
        [SerializeField] private VehicleSetup vehicleSetup;

        public bool RunSumoInBackground => optimizationSettings.RunSumoInBackground;
        private SocketConnector SocketConnector;





        // ======================
        //      Start
        // ======================

        void Start()
        {
            SocketConnector = new SocketConnector();
            Debug.Log("Starting Client with " + SocketConnector.connectionIP + " on port " + SocketConnector.connectionPort);
            SocketConnector.Start();
        }

        void Update()
        {
            // ======================
            //      Receive Data
            // ====================== 
            debugData.messageReceived = SocketConnector.messageReceived;

            if (debugData.messageReceived != null)
            {
                DeserializeStepInfo(debugData.messageReceived);
                UpdateVehiclesDictionary();
            }        

            // ======================
            //      Send Data
            // ======================

            if (simulatorVehicleInfo._sendData)
            {
                SerializableVehicle ego = new SerializableVehicle();
                ego.id = simulatorVehicleInfo.egoVehicleId;

                Vector3 egoPos = simulatorVehicleInfo.egoVehicle.position;
                float egoRot = simulatorVehicleInfo.egoVehicle.rotation.eulerAngles.y;

                ego.positionX = egoPos.x;
                ego.positionY = egoPos.z;
                ego.rotation = egoRot;

                debugData.messageToSend = JsonConvert.SerializeObject(ego);
                SocketConnector.messageToSend = debugData.messageToSend;
            }
        }

        public void DeserializeStepInfo(string message)
        {
            try
            {
                _stepInfo = JsonConvert.DeserializeObject<SumoSimulationStepInfo>(message);
            }
            catch (JsonException ex)
            {
                Debug.LogWarning($"Json Deserialization of SumoStep failed! Exception: {ex.Message}");
                _stepInfo = new SumoSimulationStepInfo();
            }
        }

        private void OnApplicationQuit()
        {
           SocketConnector.Close();
        }

        void UpdateVehiclesDictionary()
        {
            float time = _stepInfo.time;

            // check for new vehicles and Add them
            CheckForNewVehiclesAndAdd();

            // check for non existent vehicles in sumo
            RemoveNonExistentActors();
            
            // check if sumo runs in background -> delete all actors
            RemoveAllActorsIfSumoInBackgroundMode();
            
        }

        public void RemoveAllActorsIfSumoInBackgroundMode(){
            if(RunSumoInBackground)
            {

                // Destroy all vehicles
                if(vehicleSetup.vehDict.Count>0)
                {
                    List<string> keysToRemove = new List<string>();
                    //Debug.LogError(vehicleSetup.vehDict.Count);
                    foreach (KeyValuePair<string, GameObject> kvp in vehicleSetup.vehDict)
                    {  
                        if(kvp.Value.gameObject.CompareTag("Bus") && !vehicleToggles.busEnable)
                        {
                            Debug.LogWarning("Bus is not destroyed in background mode!");
                        }
                        else if(kvp.Value.gameObject.CompareTag("Ego"))
                        {
                            Debug.LogWarning("Ego is not destroyed in background mode!");
                        }
                        else if(kvp.Value.gameObject.CompareTag("Person") && !vehicleToggles.pedestrianEnable)
                        {
                            Debug.LogWarning("Person is not destroyed");
                        }
                        else
                        {
                            Debug.Log("Destroyed Vehicle = " + kvp.Value.gameObject.name);   
                            Destroy(kvp.Value.gameObject);
                            keysToRemove.Add(kvp.Key);
                            
                        }
                    }

                    foreach (string key in keysToRemove)
                    {
                        vehicleSetup.vehDict.Remove(key);
                    }
                    //vehicleSetup.vehDict.Clear();
                }
            }
        }

        public void CheckForNewVehiclesAndAdd(){
            foreach (SerializableVehicle serVehicle in _stepInfo.vehicleList)
            {   
                //check for every vehicle if
                if(!RunSumoInBackground)
                {
                    string vehId = serVehicle.id;

                    bool containsKey = vehicleSetup.vehDict.ContainsKey(vehId);

                    bool isInRadius = true;
                    if(optimizationSettings.useEgoRadius)
                    {
                        isInRadius = Vector3.Distance(new Vector3(serVehicle.positionX, 0, serVehicle.positionY), 
                            simulatorVehicleInfo.egoVehicle.position) <= optimizationSettings.egoRadius;
                    }
                    
                    // If dictionary does not contain key spawn a vehicle. Do this only if it's not the ego vehicle 
                    // and if the vehicle is in the defined radius (for the default case the radius can be seen as infite)
                    if (!containsKey && !(vehId == simulatorVehicleInfo.egoVehicleId) && isInRadius)
                    {
                        GameObject vehObj;
                        float specificHeightOfCoordianteFrame;

                        if (serVehicle.vehicleType == "passenger")
                        {
                            vehObj = vehicleSetup.vehicleCompositions.PassengerCars[Random.Range(0, vehicleSetup.vehicleCompositions.PassengerCars.Count)];
                            specificHeightOfCoordianteFrame = 0f;
                        }
                        else if (serVehicle.vehicleType == "bicycle")
                        {
                            vehObj = vehicleSetup.vehicleCompositions.Bicycles[Random.Range(0, vehicleSetup.vehicleCompositions.Bicycles.Count)];
                            specificHeightOfCoordianteFrame = 0f;
                        }
                        else if (serVehicle.vehicleType == "pedestrian" && !vehicleToggles.pedestrianEnable)
                        {
                            vehObj = vehicleSetup.vehicleCompositions.Persons[Random.Range(0, vehicleSetup.vehicleCompositions.Persons.Count)];
                            specificHeightOfCoordianteFrame = 1.1f;
                        }
                        // check if bus is enabled
                        else if (serVehicle.vehicleType == "bus" && !vehicleToggles.busEnable)
                        {
                            vehObj = vehicleSetup.vehicleCompositions.Busses[Random.Range(0, vehicleSetup.vehicleCompositions.Busses.Count)];
                            specificHeightOfCoordianteFrame = 1f;
                        }
                        else if (serVehicle.vehicleType == "taxi")
                        {
                            vehObj = vehicleSetup.vehicleCompositions.Taxis[Random.Range(0, vehicleSetup.vehicleCompositions.Taxis.Count)];
                            specificHeightOfCoordianteFrame = 1f;
                        }
                        else
                        {
                            Debug.LogWarning("Vehicle Type not found in Vehicle Compositions Scriptable Object!");
                            continue;
                        }

                        Vector3 pos = new Vector3(serVehicle.positionX, specificHeightOfCoordianteFrame, serVehicle.positionY); // veh is starting at 1m height as there is the coordinate frame
                        Quaternion rot = Quaternion.Euler(0, serVehicle.rotation+(float)180, 0);
                        //before spawning 
                        GameObject veh = Instantiate(vehObj,pos,rot);

                        veh.name = serVehicle.id + "-" + serVehicle.vehicleType + "-" + vehObj.name;

                        Color vehcolor = Random.ColorHSV(); // Random color for the vehicle
                        foreach (Transform child in veh.transform)
                        {
                            GameObject bodyComponent = child.Find("Body")?.gameObject;
                            if (bodyComponent != null)
                            {
                                foreach (Transform child2 in child)
                                {
                                    MeshRenderer meshRenderer = child2.GetComponent<MeshRenderer>();
                                    if (meshRenderer != null && meshRenderer.materials.Length > 0)
                                    {
                                        foreach (Material material in meshRenderer.materials)
                                        {
                                            if (material.name == child.name + "_Body (Instance)") // only changes the color of the body
                                            {
                                                material.color = vehcolor;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        IVehicleController controller;
                        controller = veh.GetComponent<IVehicleController>();
                        if (controller == null)
                        {
                            // try subcomponents
                            controller = veh.GetComponentInChildren<IVehicleController>();
                            if (controller == null)
                            {
                                Debug.LogError("Vehicle Controller not found!");
                                continue;
                            }
                        }
                        controller.id = serVehicle.id;

                        vehicleSetup.vehDict.Add(serVehicle.id, veh);

                    }
                    
                }
            }
        }

        public void RemoveNonExistentActors()
        {
            try
            {
                foreach (KeyValuePair<string, GameObject> kvp in vehicleSetup.vehDict)
                {
                    bool idIsContained = false;
                    foreach (SerializableVehicle serVehicle in _stepInfo.vehicleList)
                    {
                        if (kvp.Key == serVehicle.id)
                        {
                            idIsContained = true;
                            break;
                        }
                    }

                    if (!idIsContained)
                    {
                        Destroy(kvp.Value.gameObject);
                        vehicleSetup.vehDict.Remove(kvp.Key);
                    }
                }
            }
            catch (System.Exception e)
            {
                // Handle the exceptions
                Debug.LogWarning("An exception occurred: " + e.Message);
                // Something is wrong here. Time to fix it later TODO
            }
            finally
            {
                // Code that always runs, even if an exception is thrown
                // Debug.LogWarning("Cleanup code in the finally block.");
            }
        }

        public void SetRunSumoInBackground(bool value)
        {
            optimizationSettings.RunSumoInBackground = value;
        }

        public void SetBusEnable(bool value)
        {
            vehicleToggles.busEnable = value;
        }

        public void SetPedestrianEnable(bool value)
        {
            vehicleToggles.pedestrianEnable = value;
        }

        private void OnDrawGizmos()
        {
            if (debugData.showDebugGizmoSimulatorEgo)
            {
                Gizmos.color = new Color(1, 0, 0, 0.5f);
                Gizmos.DrawSphere(simulatorVehicleInfo.egoVehicle.position, 2);
                Gizmos.DrawSphere(simulatorVehicleInfo.egoVehicle.position, optimizationSettings.egoRadius);
            }
        }
    }
}
