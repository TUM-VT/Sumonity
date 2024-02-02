using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace tumvt.sumounity
{
    [System.Serializable]
    public class SumoSimulationStepInfo
    {
        public float time;
        public List<SerializableVehicle> vehicleList = new List<SerializableVehicle>();
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
            _values = new VehicleState(pos, rot, speed, signals, time);
        }

        public void UpdateVehicleValues(SerializableVehicle serVeh, float time)
        {
            Vector3 pos = new Vector3(serVeh.positionX, 0, serVeh.positionY);
            Quaternion rot = Quaternion.Euler(0, serVeh.rotation, 0);
            float speed = serVeh.speed;
            int signals = serVeh.signals;

            if (useInterpolation)
            {

            }

            if (!useInterpolation)
            {
                _lastValues = _values;
                _values = new VehicleState(pos, rot, speed, signals, time);
            }


            // Update Prefab Position
            SetVehicleState(_values);

        }


        void SetVehicleState(VehicleState val)
        {
            _gameObject.transform.position = val._position;
            _gameObject.transform.rotation = val._rotation;
            //....
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


        public static (float,float,float) SumoVehicleControl(ref SumoSocketClient sock, string id, Rigidbody rb, float steeringGain, ref PIDController pidControllerSpeed, ref PIDController pidControllerDist, ref Vector2 lookAheadMarker){
            // Get info from SUMO Stepinfo object
            SerializableVehicle currentVehicleState=null;
            foreach(SerializableVehicle veh in sock.stepInfo.vehicleList)
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



    }

    [System.Serializable]
    public class VehicleState
    {
        public Vector3 _position;
        public Quaternion _rotation;
        public float _speed;
        public int _signals;
        public float _time;

        public VehicleState(Vector3 pos, Quaternion rot, float speed, int signal, float time)
        {
            _position = pos;
            _rotation = rot;
            _speed = speed;
            _signals = signal;
            _time = time;
        }
    }
}
