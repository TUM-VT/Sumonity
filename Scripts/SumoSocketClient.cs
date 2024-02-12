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
        public string messageReceived;
        public string messageToSend;
        private string egoVehicleId = "bike1";
        public SumoSimulationStepInfo stepInfo;

        public Transform EgoVehicle;

        private SocketClient socketClient;


        [Header("Vehicles")]
        public VehicleCompositionsScriptableObject vehicleCompositions; 
        public Dictionary<string, GameObject> vehDict = new Dictionary<string, GameObject>();     



        [Header("Misc")]
        public bool sendData;

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


        void UpdateVehiclesDictionary()
        {
            float time = stepInfo.time;

            foreach (SerializableVehicle serVehicle in stepInfo.vehicleList)
            {
                string vehId = serVehicle.id;

                bool containsKey = vehDict.ContainsKey(vehId);


                if (!containsKey && !(vehId == egoVehicleId))
                { // If dictionary does not contain key
                    
                    GameObject vehObj;
                    float specificHeightOfCoordianteFrame;
                    
                    if(serVehicle.vehicleType == "passenger")
                    {
                        vehObj = vehicleCompositions.PassengerCars[Random.Range(0, vehicleCompositions.PassengerCars.Count)];
                        specificHeightOfCoordianteFrame = 1f;
                    }
                    else if(serVehicle.vehicleType == "bicycle")
                    {
                        vehObj = vehicleCompositions.Bicycles[Random.Range(0, vehicleCompositions.Bicycles.Count)];
                        specificHeightOfCoordianteFrame = 0f;
                    }
                    else if(serVehicle.vehicleType == "pedestrian")
                    {
                        vehObj = vehicleCompositions.Persons[Random.Range(0, vehicleCompositions.Persons.Count)];
                        specificHeightOfCoordianteFrame = 1.1f;
                    }
                    else if(serVehicle.vehicleType == "bus")
                    {
                        vehObj = vehicleCompositions.Busses[Random.Range(0, vehicleCompositions.Busses.Count)];
                        specificHeightOfCoordianteFrame = 1f;
                    }
                    else if(serVehicle.vehicleType == "taxi")
                    {
                        vehObj = vehicleCompositions.Taxis[Random.Range(0, vehicleCompositions.Taxis.Count)];
                        specificHeightOfCoordianteFrame = 1f;
                    }                    
                    else
                    {
                        Debug.LogWarning("Vehicle Type not found in Vehicle Compositions Scriptable Object!");
                        continue;
                    }

                    vehObj.name = serVehicle.id + " - " + serVehicle.vehicleType;

                    Vector3 pos = new Vector3(serVehicle.positionX, specificHeightOfCoordianteFrame, serVehicle.positionY); // veh is starting at 1m height as there is the coordinate frame
                    Quaternion rot = Quaternion.Euler(0, serVehicle.rotation+(float)180, 0);
                    GameObject veh = Instantiate(vehObj,pos,rot);
                    
                    IVehicleController controller;
                    controller = veh.GetComponent<IVehicleController>();
                    if (controller == null)
                    {
                        // try subcomponents
                        controller = veh.GetComponentInChildren<IVehicleController>();
                        if (controller == null){
                            Debug.LogError("Vehicle Controller not found!");
                            continue;
                        }
                    }
                    controller.id = serVehicle.id;

                    vehDict.Add(serVehicle.id, veh);

                }
            }

            // check for non existent vehicles in sumo

            try
            {
                foreach(KeyValuePair<string, GameObject> kvp in vehDict)
                {
                    bool idIsContained = false;
                    foreach (SerializableVehicle serializableVehicle in stepInfo.vehicleList)
                    {
                        if (kvp.Key == serializableVehicle.id)
                            idIsContained = true;                
                    }

                    if(idIsContained)
                    {
                        // Check for position in BBOX
                        // if false
                        // destroy
                    }

                    if(!idIsContained)
                    {
                        Destroy(kvp.Value.gameObject);
                        vehDict.Remove(kvp.Key);
                    }
                }
            }
            catch (System.Exception e)
            {
                // Handle the exceptions
                // Debug.LogWarning("An exception occurred: " + e.Message);
                // Something is wrong here. Time to fix it later TODO
            }
            finally
            {
                // Code that always runs, even if an exception is thrown
                // Debug.LogWarning("Cleanup code in the finally block.");
            }


            
        }

    }
}
