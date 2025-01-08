using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;

namespace tumvt.sumounity
{
    [System.Serializable]
    public class SumoSimulationStepInfo
    {
        public float time;
        public List<SerializableVehicle> vehicleList = new List<SerializableVehicle>();
        public List<Junction> junctionList = new List<Junction>();        
    }

    [System.Serializable]
    public class SerializableVehicle
    { // Vehicle class only for Json Serialization
        public string id;
        public string vehicleType;
        public float positionX;
        public float positionY;
        public float rotation;
        public float speed;
        public int signals;
        public float lookaheadPosX;
        public float lookaheadPosY;
        public int stopState;
    }

    [System.Serializable]
    public class Junction
    {
        public int id;
        public float[] pos;
        public int phase;
        public string state;
        public List<List<float>> armPos;
    }


    [System.Serializable]
    public class Vehicle
    {
        public string _id;
        public string _vehicleType;
        public string _name;
        public GameObject _gameObject;
        public bool useInterpolation;

        public VehicleState _values;

        public VehicleState _lastValues;
        public VehicleState _nextValues;

        public Vehicle(SerializableVehicle serVeh, float time)
        {
            _id = serVeh.id;
            _vehicleType = serVeh.vehicleType;
            _name = _id + " (" + _vehicleType + ")";


            Vector3 pos = new Vector3(serVeh.positionX, 0, serVeh.positionY);
            Quaternion rot = Quaternion.Euler(0, serVeh.rotation, 0);
            float speed = serVeh.speed;
            int signals = serVeh.signals;
            int stopState = serVeh.stopState;
            _values = new VehicleState(pos, rot, speed, signals, time, stopState);
        }

        public void UpdateVehicleValues(SerializableVehicle serVeh, float time)
        {
            Vector3 pos = new Vector3(serVeh.positionX, 0, serVeh.positionY);
            Quaternion rot = Quaternion.Euler(0, serVeh.rotation, 0);
            float speed = serVeh.speed;
            int signals = serVeh.signals;
            int stopState = serVeh.stopState;

            if (useInterpolation)
            {

            }

            if (!useInterpolation)
            {
                _lastValues = _values;
                _values = new VehicleState(pos, rot, speed, signals, time, stopState);
            }


            // Update Prefab Position
            SetVehicleState(_values);

        }


        void SetVehicleState(VehicleState val)
        {
            _gameObject.transform.position = val._position;
            _gameObject.transform.rotation = val._rotation;
            //....[System.Serializable]
        }

        static Vector2 RotateVectorByAngle(Vector2 v, float degrees) 
        {
            float radians = degrees * Mathf.Deg2Rad;
            float sin = Mathf.Sin(radians);
            float cos = Mathf.Cos(radians);

            float tx = v.x;
            float ty = v.y;

            return new Vector2(cos * tx - sin * ty, sin * tx + cos * ty);
        }

        // this will be used in the future to optimize simulation speed for user studies.
        public static bool SumoVehicleDetect(ref SumoSocketClient sock, string id){
            // Get info from SUMO Stepinfo object
            // bool isInsidePhsyicsArea=false;	
            // bool VehicleFound=false;
            // SerializableVehicle currentVehicleState=null;
            // foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            // {
            //     if(id == veh.id && veh.lookaheadPosX == -999)
            //     {
            //         isInsidePhsyicsArea = false;
            //         VehicleFound=true;
            //         break;
            //     }
            //     else if(id == veh.id && veh.lookaheadPosX != -999)
            //     {
            //         isInsidePhsyicsArea = true;
            //         VehicleFound=true;
            //         break;
            //     }
            // }
            // if (VehicleFound==false)
            //     Debug.LogWarning("There is no sumo object with id: "+id);

            bool isInsidePhsyicsArea = true;
            return isInsidePhsyicsArea;

        }
        public static Vector2 SUMO_groundtruth_back(ref SumoSocketClient sock, string id){
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState=null;
            foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if(id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState==null)
                Debug.LogWarning("There is no sumo object with id: "+id);

            Vector2 sumoPosition = new Vector2(currentVehicleState.positionX,currentVehicleState.positionY);
            return sumoPosition;
        }

        public static Rigidbody SumoVehicleTeleport(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker)
        {
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState = null;
            foreach (SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if (id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState == null)
            {
                Debug.LogWarning("There is no sumo object with id: " + id);
                return rb;
            }

            Rigidbody rb_mod = rb;

            // Set the new position
            Vector3 newPosition = rb_mod.position;
            newPosition.x = currentVehicleState.positionX;
            newPosition.z = currentVehicleState.positionY;
            rb_mod.MovePosition(newPosition);

            // Set the new rotation
            Vector3 newRotation = rb_mod.rotation.eulerAngles;
            newRotation.y = currentVehicleState.rotation - 180;
            rb_mod.rotation = Quaternion.Euler(newRotation);

            return rb_mod;
        }

        public static Rigidbody SumoTaxiTeleport(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker)
        {
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState = null;
            foreach (SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if (id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState == null)
            {
                Debug.LogWarning("There is no sumo object with id: " + id);
                return rb;
            }

            Rigidbody rb_mod = rb;
            //Debug.LogError("currentVehicleState.positionX: " + currentVehicleState.positionX);
            // Set the new position
            Vector3 newPosition = rb_mod.position;
            newPosition.x = currentVehicleState.positionX;
            //Debug.LogError("currentVehicleState.positionY: "+currentVehicleState.positionY);
            newPosition.z = currentVehicleState.positionY;
            //Debug.LogError("newPosition: "+newPosition);
            rb_mod.transform.position = newPosition;
            //Debug.LogError("rb_mod.position: "+rb_mod.position);

            // Set the new rotation
            Vector3 newRotation = rb_mod.rotation.eulerAngles;
            newRotation.y = currentVehicleState.rotation - 180;
            rb_mod.rotation = Quaternion.Euler(newRotation);

            return rb_mod;
        }

        
        public static Rigidbody SumoBicycleTeleport(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker){
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState=null;
            foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if(id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState==null)
                Debug.LogWarning("There is no sumo object with id: "+id);


            Rigidbody rb_mod = rb;
            

            Vector3 newPosition = rb_mod.position;
            newPosition.x = currentVehicleState.positionX; 
            newPosition.z = currentVehicleState.positionY;
            rb_mod.position = newPosition;  
            Vector3 newRotation = rb_mod.rotation.eulerAngles;
            newRotation.y = currentVehicleState.rotation - 180; 
            rb_mod.rotation = Quaternion.Euler(newRotation);

            return rb_mod;

        }


        public static (float,float,float) SumoVehicleControl(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker){
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState=null;
            foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if(id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState==null)
                Debug.LogWarning("There is no sumo object with id: "+id);


            Vector2 actualPos = new Vector2(rb.position.x,rb.position.z);
            Vector3 eulerAngles = rb.transform.eulerAngles;

            // We have to consider the positional error to make up for the integrating speed error.
            Vector2 targetPosLongitudinal = new Vector2(currentVehicleState.positionX,currentVehicleState.positionY);
            Vector2 targetPosLateral = new Vector2(currentVehicleState.lookaheadPosX,currentVehicleState.lookaheadPosY);
            lookAheadMarker = targetPosLateral;


            // Evaluate steering angle based on pure pursuit approach                
            float unityAngle = eulerAngles.y;    

            // ===========================
            // Steering Control (Lateral)
            // ===========================

            Vector2 translatedPointLateralControl = targetPosLateral - actualPos;
            
            // float rotationAngle = -unityAngle; // Rotation of the second coordinate frame in degrees
            // this is the position of the vehicle in sumo
            Vector2 rotatedPointLateralControl = RotateVectorByAngle(translatedPointLateralControl, unityAngle);
            // Vector2 rotatedPoint = RotateVectorByAngle(translatedPoint, -rotationAngle);
            // we want to move the point a little bit further ahead so we avoid overtaking it, which 
            // makes our lateral control unstable. (Note: longitudinal axis is y for the vehicle frame)
            Vector2 lookAheadPoint = new Vector2(rotatedPointLateralControl.x,rotatedPointLateralControl.y);
            float angleWithRespectToYAxis = Mathf.Atan2(lookAheadPoint.x, lookAheadPoint.y) * Mathf.Rad2Deg;


            float steeringValue=angleWithRespectToYAxis*steeringGain;

            
            // ===============================
            // Velocity Control (Longitudinal)
            // ===============================

            Vector2 translatedPointLongitudinalControl = targetPosLongitudinal - actualPos;
            Vector2 rotatedPointLongitudinalControl = RotateVectorByAngle(translatedPointLongitudinalControl, unityAngle);

            
            float currentVelocity = rb.velocity.magnitude;
            float desiredVelocity = currentVehicleState.speed;

            // distnace based speed control
            float setpointDistance = 0; // distance at which we want to follow the point ahead

            // unity coordinate frame is:
            // y axis points to the front 
            // x axis points to the right side of the ego vehicle (not sure about x axis ;) )
            float torqueInput1 = pidControllerSpeed.Control(desiredVelocity, currentVelocity); // positive output
            float torqueInput2 = pidControllerDist.Control(setpointDistance, rotatedPointLongitudinalControl.y)*(-1); // negative output so we multiply by -1
            
            float torqueInput = (torqueInput1+torqueInput2)/2;
            torqueInput = Mathf.Clamp(torqueInput,-1f,1f);
            
            if (false){
                // Write data to CSV file
                string filePath = "/home/ludwig-20-04/unity_ws/data/simDataVehicle.csv"; // Replace with your desired file path
                using (StreamWriter writer = new StreamWriter(filePath, true))
                {
                    string timeStamp = Time.time.ToString(); // Unity time stamp
                    string data = $"{timeStamp},{steeringValue},{torqueInput},{desiredVelocity},{rb.position.x},{rb.position.z},{currentVehicleState.positionX},{currentVehicleState.positionY},{currentVelocity}";

                    // copilot eidit my data string and include the postion of the ridigdbody and the sumo
                    writer.WriteLine(data);
                }
            }
            return (steeringValue, torqueInput, desiredVelocity);

        }

        public static (float,float,float) SumoVehicleControlWarmup(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker){
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState=null;
            foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if(id == veh.id)
                {
                    currentVehicleState = veh;
                    break;
                }
            }

            if (currentVehicleState==null)
                Debug.LogWarning("There is no sumo object with id: "+id);


            Vector2 actualPos = new Vector2(rb.position.x,rb.position.z);
            Vector3 eulerAngles = rb.transform.eulerAngles;

            // We have to consider the positional error to make up for the integrating speed error.
            Vector2 targetPosLongitudinal = new Vector2(currentVehicleState.positionX,currentVehicleState.positionY);
            Vector2 targetPosLateral = new Vector2(currentVehicleState.lookaheadPosX,currentVehicleState.lookaheadPosY);
            lookAheadMarker = targetPosLateral;


            // Evaluate steering angle based on pure pursuit approach                
            float unityAngle = eulerAngles.y;    

            // ===========================
            // Steering Control (Lateral)
            // ===========================

            Vector2 translatedPointLateralControl = targetPosLateral - actualPos;
            
            // float rotationAngle = -unityAngle; // Rotation of the second coordinate frame in degrees
            // this is the position of the vehicle in sumo
            Vector2 rotatedPointLateralControl = RotateVectorByAngle(translatedPointLateralControl, unityAngle);
            // Vector2 rotatedPoint = RotateVectorByAngle(translatedPoint, -rotationAngle);
            // we want to move the point a little bit further ahead so we avoid overtaking it, which 
            // makes our lateral control unstable. (Note: longitudinal axis is y for the vehicle frame)
            Vector2 lookAheadPoint = new Vector2(rotatedPointLateralControl.x,rotatedPointLateralControl.y);
            float angleWithRespectToYAxis = Mathf.Atan2(lookAheadPoint.x, lookAheadPoint.y) * Mathf.Rad2Deg;


            float steeringValue=angleWithRespectToYAxis*steeringGain;

            
            // ===============================
            // Velocity Control (Longitudinal)
            // ===============================

            Vector2 translatedPointLongitudinalControl = targetPosLongitudinal - actualPos;
            Vector2 rotatedPointLongitudinalControl = RotateVectorByAngle(translatedPointLongitudinalControl, unityAngle);

            float magicValue = 1.0f;

            Vector3 newVelocity = rb.transform.forward * currentVehicleState.speed*magicValue; 
            rb.velocity = newVelocity;
            float currentVelocity = rb.velocity.magnitude;
            float desiredVelocity = currentVehicleState.speed;

            // distnace based speed control
            float setpointDistance = 0; // distance at which we want to follow the point ahead

            // unity coordinate frame is:
            // y axis points to the front 
            // x axis points to the right side of the ego vehicle (not sure about x axis ;) )
            // Debug.LogWarning("Des.Velocity|currentVelocity: "+desiredVelocity+"|"+currentVelocity);
            float torqueInput1 = pidControllerSpeed.Control(desiredVelocity, currentVelocity); // positive output
            float torqueInput2 = pidControllerDist.Control(setpointDistance, rotatedPointLongitudinalControl.y)*(-1); // negative output so we multiply by -1
            
            float torqueInput = (torqueInput1+torqueInput2)/2;
            torqueInput = Mathf.Clamp(torqueInput,-1f,1f);
            
            if (false){
                // Write data to CSV file
                string filePath = "/home/ludwig-20-04/unity_ws/data/simDataVehicle.csv"; // Replace with your desired file path
                using (StreamWriter writer = new StreamWriter(filePath, true))
                {
                    string timeStamp = Time.time.ToString(); // Unity time stamp
                    string data = $"{timeStamp},{steeringValue},{torqueInput},{desiredVelocity},{rb.position.x},{rb.position.z},{currentVehicleState.positionX},{currentVehicleState.positionY},{currentVelocity}";

                    // copilot eidit my data string and include the postion of the ridigdbody and the sumo
                    writer.WriteLine(data);
                }
            }
            return (steeringValue, torqueInput, desiredVelocity);

        }

        public static int getVehicleStopState(ref SumoSocketClient sock, string id)
        {
            SerializableVehicle currentVehicle=null;
            foreach(SerializableVehicle veh in sock.StepInfo.vehicleList)
            {
                if(id == veh.id)
                {
                    currentVehicle = veh;
                    break;
                }
            }
            if (currentVehicle==null)
            {
                throw new InvalidOperationException("There is no sumo object with id: "+id);
            }
            return currentVehicle.stopState;
        }

     

    }

    [System.Serializable]
    public class VehicleState
    {
        public Vector3 _position;
        public Quaternion _rotation;
        public float _speed;
        public int _signals;
        public float _time;
        public int _stopState;


        public VehicleState(Vector3 pos, Quaternion rot, float speed, int signal, float time, int stopState)
        {
            _position = pos;
            _rotation = rot;
            _speed = speed;
            _signals = signal;
            _time = time;
            _stopState = stopState;
        }
    }
}
