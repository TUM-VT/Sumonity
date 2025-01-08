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


        [Header("Vehicle Setup")]
        [SerializeField] private VehicleSetup vehicleSetup;
        
        [Header("Vehicle Toggles")]
        [SerializeField] private VehicleToggles vehicleToggles;
        
        [Header("Optimization Settings")]
        [SerializeField] private OptimizationSettings optimizationSettings;
        
        [Header("Simulator Vehicle Info")]
        [SerializeField] private SimulatorVehicleInfo simulatorVehicleInfo;

        [Header("Simulation Step Information")]
        [SerializeField] private SumoSimulationStepInfo _stepInfo;
        public SumoSimulationStepInfo StepInfo => _stepInfo;

        public bool SendData => simulatorVehicleInfo._sendData;
        public Transform EgoVehicle => simulatorVehicleInfo.egoVehicle;

        private SocketConnector SocketConnector;


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
            SumoSocketClientHelper.RemoveAllActorsIfSumoInBackground(
                optimizationSettings.RunSumoInBackground,
                vehicleSetup, 
                vehicleToggles);

            SumoSocketClientHelper.CheckForNewVehiclesAndAdd(
                _stepInfo,
                optimizationSettings.RunSumoInBackground,
                vehicleSetup,
                simulatorVehicleInfo,
                optimizationSettings,
                vehicleToggles,
                optimizationSettings.isTeleportOnlyMode);

            SumoSocketClientHelper.RemoveNonExistentActors(
                vehicleSetup,
                _stepInfo);
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
