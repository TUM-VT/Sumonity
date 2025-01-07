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
        [Header("Socket Communication")]
        public string messageReceived;
        
        public bool sendData;
        public string messageToSend;
        
        private string egoVehicleId = "bike1";
        public Transform EgoVehicle;
        

        [Header("Serialization")]
        public SumoSimulationStepInfo stepInfo;

        

        public SocketClient socketClient;


        [Header("Vehicles Simulation")]
        public VehicleCompositionsScriptableObject vehicleCompositions; 
        public Dictionary<string, GameObject> vehDict = new Dictionary<string, GameObject>();

        [Header("Optimization")]
        [Tooltip("If enabled SUMO-Simulation runs in background and no vehicles are spawned")]
        public bool runInBackground = false;
        private bool busEnable = false;
        private bool pedestrianEnable = false;
        public bool useEgoRadius = false;
        public float egoRadius = 100;
        

        void Start()
        {
            socketClient = new SocketClient();
            Debug.Log("Starting Client with " + socketClient.connectionIP + " on port " + socketClient.connectionPort);
            socketClient.Start();
        }



        void Update()
        {
            // ======================
            //      Receive Data
            // ====================== 
            messageReceived = socketClient.messageReceived;

            if (messageReceived!=null){

                try
                {
                    stepInfo = JsonConvert.DeserializeObject<SumoSimulationStepInfo>(messageReceived);
                }
                catch (JsonException ex)
                {
                    Debug.LogWarning($"Json Deserialization of SumoStep failed! Exception: {ex.Message}");
                    stepInfo = new SumoSimulationStepInfo();
                }


                
                UpdateVehiclesDictionary();
                UpdateTrafficLight();
            }        

            // ======================
            //      Send Data
            // ======================
            // This is not used now and may be removed in the future, depending on the sumo interface structure

            if (sendData)
            {
                SerializableVehicle ego = new SerializableVehicle();
                ego.id = "bike1";

                Vector3 egoPos = EgoVehicle.position;
                // Vector3 egoPos = EgoVehicle.position + new Vector3(200, 0, 100f);
                float egoRot = EgoVehicle.rotation.eulerAngles.y;

                ego.positionX = egoPos.x;
                ego.positionY = egoPos.z;
                ego.rotation = egoRot;

                messageToSend = JsonConvert.SerializeObject(ego);
                socketClient.messageToSend = messageToSend;
            }
        }


        private void OnApplicationQuit()
        {
           socketClient.Close();
        }

        void UpdateTrafficLight()
        {
            ;
        }

        void UpdateVehiclesDictionary()
        {

            float time = stepInfo.time;

            foreach (SerializableVehicle serVehicle in stepInfo.vehicleList)
            {   
                //check for every vehicle if its a bus, a pedestrian or if backgorund mode is off
                if(serVehicle.vehicleType == "bus" || !runInBackground || serVehicle.vehicleType == "pedestrian")
                {
                    string vehId = serVehicle.id;

                    bool containsKey = vehDict.ContainsKey(vehId);

                    bool isInRadius = true;
                    if(useEgoRadius)
                    {
                        isInRadius = Vector3.Distance(new Vector3(serVehicle.positionX, 0, serVehicle.positionY), EgoVehicle.position) <= egoRadius;

                    }
                    
                    if (!containsKey && !(vehId == egoVehicleId) && isInRadius)
                    { // If dictionary does not contain key spawn a vehicle. Do this only if it's not the ego vehicle and if the vehicle is in the defined radius (for the default case the radius can be seen as infite)

                        GameObject vehObj;
                        float specificHeightOfCoordianteFrame;

                        if (serVehicle.vehicleType == "passenger")
                        {
                            vehObj = vehicleCompositions.PassengerCars[Random.Range(0, vehicleCompositions.PassengerCars.Count)];
                            specificHeightOfCoordianteFrame = 0f;
                        }
                        else if (serVehicle.vehicleType == "bicycle")
                        {
                            vehObj = vehicleCompositions.Bicycles[Random.Range(0, vehicleCompositions.Bicycles.Count)];
                            specificHeightOfCoordianteFrame = 0f;
                        }
                        else if (serVehicle.vehicleType == "pedestrian" && !pedestrianEnable)
                        {
                            vehObj = vehicleCompositions.Persons[Random.Range(0, vehicleCompositions.Persons.Count)];
                            specificHeightOfCoordianteFrame = 1.1f;
                        }
                        // check if bus is enabled
                        else if (serVehicle.vehicleType == "bus" && !busEnable)
                        {
                            vehObj = vehicleCompositions.Busses[Random.Range(0, vehicleCompositions.Busses.Count)];
                            specificHeightOfCoordianteFrame = 1f;
                        }
                        else if (serVehicle.vehicleType == "taxi")
                        {
                            vehObj = vehicleCompositions.Taxis[Random.Range(0, vehicleCompositions.Taxis.Count)];
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

                        vehDict.Add(serVehicle.id, veh);

                    }
                    
                }
            }

            // check for non existent vehicles in sumo
            try
            {
                foreach (KeyValuePair<string, GameObject> kvp in vehDict)
                {
                    bool idIsContained = false;
                    foreach (SerializableVehicle serVehicle in stepInfo.vehicleList)
                    {
                        if (kvp.Key == serVehicle.id)
                        {
                            idIsContained = true;
                            break;
                        }
                    }
                    if (idIsContained && useEgoRadius)
                    {
                        // Check for position in BBOX
                        // if false
                        // destroy
                        foreach (SerializableVehicle serVehicle in stepInfo.vehicleList)
                        {
                            if (Vector3.Distance(new Vector3(serVehicle.positionX, 0, serVehicle.positionY), EgoVehicle.position) <= egoRadius)
                            { // Check Distance to EgoVehicle
                                Destroy(kvp.Value.gameObject);
                                vehDict.Remove(kvp.Key);
                            }
                        }
                    }
                    if (!idIsContained)
                    {
                        Destroy(kvp.Value.gameObject);
                        vehDict.Remove(kvp.Key);
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
            
            if(runInBackground)
            {

                // Destroy all vehicles
                if(vehDict.Count>0)
                {
                    List<string> keysToRemove = new List<string>();
                    //Debug.LogError(vehDict.Count);
                    foreach (KeyValuePair<string, GameObject> kvp in vehDict)
                    {  
                        if(kvp.Value.gameObject.CompareTag("Bus") && !busEnable)
                        {
                            Debug.LogWarning("Bus is not destroyed in background mode!");
                        }
                        else if(kvp.Value.gameObject.CompareTag("Ego"))
                        {
                            Debug.LogWarning("Ego is not destroyed in background mode!");
                        }
                        else if(kvp.Value.gameObject.CompareTag("Person") && !pedestrianEnable)
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
                        vehDict.Remove(key);
                    }
                    //vehDict.Clear();
                }
            }
            
        }

        public void SetRunInBackground(bool value)
        {
            runInBackground = value;
        }

        public void SetBusEnable(bool value)
        {
            busEnable = value;
        }

        public void SetPedestrianEnable(bool value)
        {
            pedestrianEnable = value;
        }

        private void OnDrawGizmos()
        {
            if (useEgoRadius)
            {
                Gizmos.color = new Color(1, 0, 0, 0.5f);
                Gizmos.DrawSphere(EgoVehicle.position, 2);
                Gizmos.DrawSphere(EgoVehicle.position, egoRadius);
            }
        }
    }
}
