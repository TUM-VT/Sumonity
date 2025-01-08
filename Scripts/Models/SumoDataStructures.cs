using UnityEngine;
using System.Collections.Generic;

namespace tumvt.sumounity
{
    [System.Serializable]
    public class VehicleSetup
    {
        public VehicleCompositionsScriptableObject vehicleCompositions;
        public Dictionary<string, GameObject> vehDict = new Dictionary<string, GameObject>();
    }

    [System.Serializable]
    public class VehicleToggles
    {
        public bool busEnable = false;
        public bool pedestrianEnable = false;
    }

    [System.Serializable]
    public class OptimizationSettings
    {
        [Tooltip("If enabled SUMO-Simulation runs in background and no vehicles are spawned")]
        public bool RunSumoInBackground = false;
        public bool useEgoRadius = false;
        [Range(0, 500)] 
        public float egoRadius = 100;
    }

    [System.Serializable]
    public class SimulatorVehicleInfo
    {
        [Tooltip("The ID of the ego vehicle in SUMO used for inserting vehicles into the SUMO simulation")]
        public string egoVehicleId = "bike1";
        [Tooltip("The Transform of the ego vehicle in Unity used for inserting vehicles into the SUMO simulation")]
        public Transform egoVehicle;
        [Tooltip("Send data to SUMO")]
        public bool _sendData;
    }
} 