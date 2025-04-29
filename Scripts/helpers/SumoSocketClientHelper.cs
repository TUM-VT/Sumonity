using UnityEngine;
using System.Collections.Generic;
using tum_car_controller;

namespace tumvt.sumounity
{
    public static class SumoSocketClientHelper
    {
        public static void RemoveAllActorsIfSumoInBackground(
            bool runInBackground,
            VehicleSetup vehicleSetup,
            VehicleToggles vehicleToggles)
        {
            if(!runInBackground) return;

            if(vehicleSetup.vehDict.Count <= 0) return;

            List<string> keysToRemove = new List<string>();
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
                    Object.Destroy(kvp.Value.gameObject);
                    keysToRemove.Add(kvp.Key);
                }
            }

            foreach (string key in keysToRemove)
            {
                vehicleSetup.vehDict.Remove(key);
            }
        }

        public static void CheckForNewVehiclesAndAdd(
            SumoSimulationStepInfo stepInfo,
            bool runInBackground,
            VehicleSetup vehicleSetup,
            SimulatorVehicleInfo simulatorVehicleInfo,
            OptimizationSettings optimizationSettings,
            VehicleToggles vehicleToggles,
            bool isTeleportOnlyMode)
        {
            if(runInBackground) return;

            foreach (SerializableVehicle serVehicle in stepInfo.vehicleList)
            {   
                string vehId = serVehicle.id;
                if (vehicleSetup.vehDict.ContainsKey(vehId)) continue;
                if (vehId == simulatorVehicleInfo.egoVehicleId) continue;

                bool isInRadius = true;
                if(optimizationSettings.useEgoRadius)
                {
                    isInRadius = Vector3.Distance(
                        new Vector3(serVehicle.positionX, 0, serVehicle.positionY), 
                        simulatorVehicleInfo.egoVehicle.position) <= optimizationSettings.egoRadius;
                }

                if (!isInRadius) continue;

                SpawnNewVehicle(serVehicle, vehicleSetup, vehicleToggles, isTeleportOnlyMode);
            }
        }

        private static void SpawnNewVehicle(
            SerializableVehicle serVehicle, 
            VehicleSetup vehicleSetup,
            VehicleToggles vehicleToggles,
            bool isTeleportOnlyMode)
        {
            GameObject vehObj;
            float specificHeightOfCoordianteFrame;

            (vehObj, specificHeightOfCoordianteFrame) = GetVehicleObjectAndHeight(
                serVehicle.vehicleType, 
                vehicleSetup, 
                vehicleToggles);

            if (vehObj == null) return;

            Vector3 pos = new Vector3(
                serVehicle.positionX, 
                specificHeightOfCoordianteFrame+2f, // additional height, so cars drop onto the road plane
                serVehicle.positionY);
            Quaternion rot = Quaternion.Euler(0, serVehicle.rotation + 180f, 0);
            
            GameObject veh = Object.Instantiate(vehObj, pos, rot);
            veh.name = $"{serVehicle.id}-{serVehicle.vehicleType}-{vehObj.name}";

            // Set the tag to "Vehicle" for all vehicles
            veh.tag = "Vehicle";

            // TODO: fix implementation to work with general vehicle controllers
            // CarController carController = veh.GetComponent<CarController>();
            // if (carController != null)
            // {
            //     carController.SetTeleportOnlyMode(isTeleportOnlyMode);
            // }

            ApplyRandomColor(veh);
            SetupVehicleController(veh, serVehicle.id);

            vehicleSetup.vehDict.Add(serVehicle.id, veh);
        }

        private static (GameObject obj, float height) GetVehicleObjectAndHeight(
            string vehicleType, 
            VehicleSetup vehicleSetup,
            VehicleToggles vehicleToggles)
        {
            switch (vehicleType)
            {
                case "passenger":
                    return (vehicleSetup.vehicleCompositions.PassengerCars[Random.Range(0, vehicleSetup.vehicleCompositions.PassengerCars.Count)], 0f);
                case "bicycle":
                    return (vehicleSetup.vehicleCompositions.Bicycles[Random.Range(0, vehicleSetup.vehicleCompositions.Bicycles.Count)], 0f);
                case "pedestrian" when !vehicleToggles.pedestrianEnable:
                    return (vehicleSetup.vehicleCompositions.Persons[Random.Range(0, vehicleSetup.vehicleCompositions.Persons.Count)], 1.1f);
                case "bus" when !vehicleToggles.busEnable:
                    return (vehicleSetup.vehicleCompositions.Busses[Random.Range(0, vehicleSetup.vehicleCompositions.Busses.Count)], 1f);
                case "taxi":
                    return (vehicleSetup.vehicleCompositions.Taxis[Random.Range(0, vehicleSetup.vehicleCompositions.Taxis.Count)], 1f);
                default:
                    Debug.LogWarning("Vehicle Type not found in Vehicle Compositions Scriptable Object!");
                    return (null, 0f);
            }
        }

        private static void ApplyRandomColor(GameObject vehicle)
        {
            Color vehcolor = Random.ColorHSV();
            foreach (Transform child in vehicle.transform)
            {
                GameObject bodyComponent = child.Find("Body")?.gameObject;
                if (bodyComponent == null) continue;

                foreach (Transform child2 in child)
                {
                    MeshRenderer meshRenderer = child2.GetComponent<MeshRenderer>();
                    if (meshRenderer == null || meshRenderer.materials.Length == 0) continue;

                    foreach (Material material in meshRenderer.materials)
                    {
                        if (material.name == child.name + "_Body (Instance)")
                        {
                            material.color = vehcolor;
                        }
                    }
                }
            }
        }

        private static void SetupVehicleController(GameObject vehicle, string vehicleId)
        {
            IVehicleController controller = vehicle.GetComponent<IVehicleController>() 
                ?? vehicle.GetComponentInChildren<IVehicleController>();

            if (controller == null)
            {
                Debug.LogError("Vehicle Controller not found!");
                return;
            }

            controller.id = vehicleId;
        }

        public static void RemoveNonExistentActors(
            VehicleSetup vehicleSetup,
            SumoSimulationStepInfo stepInfo)
        {
            try
            {
                var vehiclesToRemove = new List<string>();
                foreach (var kvp in vehicleSetup.vehDict)
                {
                    if (!stepInfo.vehicleList.Exists(v => v.id == kvp.Key))
                    {
                        Object.Destroy(kvp.Value.gameObject);
                        vehiclesToRemove.Add(kvp.Key);
                    }
                }

                foreach (string key in vehiclesToRemove)
                {
                    vehicleSetup.vehDict.Remove(key);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"An exception occurred: {e.Message}");
            }
        }
    }
} 