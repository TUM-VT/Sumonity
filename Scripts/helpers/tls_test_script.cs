using UnityEngine;
using tumvt.sumounity;
using System.Collections.Generic;
using System.Linq;

public class tls_test_script : MonoBehaviour
{
    public GameObject targetGameObject;
    public string componentName;
    public Dictionary<Transform, Junction> junctionDict = new Dictionary<Transform, Junction>(); // matches each unity TL node to a junction
    public Dictionary<Transform, int> armDict = new Dictionary<Transform, int>(); // matches each unity TL node to a junction arm
    
    private Light redLight;
    private Light greenLight;
    private Light yellowLight;

    //public int counter = 0;

    SumoSocketClient sock;

    void Start()
    {
        // get the socketclient with the step info
        // sock = GameObject.FindWithTag("GameController").GetComponent<SumoSocketClient>();
        sock = GameObject.FindObjectOfType<SumoSocketClient>();
        junctionDict.Clear();
        armDict.Clear();
    }

    void Update()
    {
        if (junctionDict.Count == 0) // only runs once at the start of simulation
        {
            FindJunction(targetGameObject.transform,0);
            FindArm();        
        }
        else UpdateTrafficLightStates();

    }
    
    // Assigns unity traffic lights to junction arm from SUMO
    void FindArm()
    {
        // groups unity traffic lights with the same arm (orientation) and junction
        var groupedTransforms = junctionDict.GroupBy(pair => new {Orientation = pair.Key.rotation, JunctionID = pair.Value.id});
        Debug.Log("Grouped Transforms: " + groupedTransforms.Count()); 
        foreach (var group in groupedTransforms)
        {

            // Calculate mean positions for each unity arm
            int n = 0;
            float x_coord = 0;
            float y_coord = 0;
            foreach (var pair in group)
            {
                n += 1;
                x_coord += pair.Key.position.x;
                y_coord += pair.Key.position.z;
            }
            float x_mean = x_coord/n;
            float y_mean = y_coord/n;

            // Find closest arm in SUMO
            int closestArmIndex = 0;
            float minDistance = float.MaxValue;

            foreach (var jxn in sock.StepInfo.junctionList)
            {
                if (jxn.id == group.Key.JunctionID)
                {
                    for (int i = 0; i < jxn.armPos.Count; i++)
                    {
                        
                        float distance = Vector2.Distance(new Vector2(x_mean, y_mean), new Vector2(jxn.armPos[i][0], jxn.armPos[i][1]));
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            closestArmIndex = i;
                        }
                    }                    
                }
            }

            // Append closest arm to Dictionary
            foreach(var pair in group)
            {
                armDict.Add(pair.Key,closestArmIndex);
            }
        }

    }

    // Assigns unity traffic lights to junctions from SUMO
    void FindJunction(Transform parentTransform, int depth)
    {
        Transform[] transforms = parentTransform.GetComponentsInChildren<Transform>(); 
        foreach (Transform transform in transforms)
        {
            if (transform.name.Substring(0,8) == "RootNode" && transform.childCount == 3) // Looks through all components of tum_main for components that begin with "RootNode" and have exactly 3 children (for red, green, yellow)
            {
                foreach (var junction in sock.StepInfo.junctionList)
                {
                    // Calculate distance between component and junction 
                    float distance = Vector2.Distance(new Vector2(transform.position.x, transform.position.z),
                                                    new Vector2(junction.pos[0], junction.pos[1]));
                    // Check if the distance is within 15 meters
                    if (distance <= 20f)
                    {   
                        junctionDict.Add(transform,junction);
                        
                        break;
                    }
                }

            }
        }
    }

    
    // Checks the current state of an TL Node in Unity 
    void UpdateTrafficLightStates()
    {
        //counter = 0;
        foreach (var pair in junctionDict)
        {
            Transform junctionTransform = pair.Key;
            int armIndex = armDict[junctionTransform];
            int junctionID = pair.Value.id;
            Junction junction = sock.StepInfo.junctionList.FirstOrDefault(j => j.id == junctionID);
            char armState = junction.state[armIndex];
            CheckPointLights(junctionTransform, armState);
        }
        //Debug.Log("Lights Switched: " + counter);

    }
    
    // Finds the point lights that corresponds to a TL 
    void CheckPointLights(Transform parentTransform, char armState)
    {
        Component[] components = parentTransform.GetComponentsInChildren<Component>(true);

        redLight = null;
        yellowLight = null;
        greenLight = null;
        Transform redNode = null;
        Transform greenNode = null;
        Transform yellowNode = null;

        foreach (Component component in components)
        {
                if (component.name == "red_light")
                {
                    redLight = component.GetComponent<Light>();
                    redNode = FindNode(component.transform, "_on");
                }
                else if (component.name == "green_light")
                {
                    greenLight = component.GetComponent<Light>();
                    greenNode = FindNode(component.transform, "_on");
                }
                else if (component.name == "yellow_light")
                {
                    yellowLight = component.GetComponent<Light>();
                    yellowNode = FindNode(component.transform, "_on");
                }

        }
        
        if (redLight == null || yellowLight == null || greenLight == null) 
        {
            AddPointLight("red_light", Color.red, parentTransform);
            AddPointLight("green_light", Color.green,parentTransform);
            AddPointLight("yellow_light", Color.yellow,parentTransform);
            
        }
        else
        {
            SwitchLights(redLight,greenLight,yellowLight, armState, redNode, greenNode, yellowNode);
            //counter += 1;
        }
        
    }

    // Creates Point Lights
    void AddPointLight(string lightName, Color color,Transform parentTransform)
    {

        foreach (Transform child in parentTransform)
        {
            if (child.name.Contains(lightName.Substring(0, lightName.Length - 6))) // Looks for the correct color light node
            {
                GameObject pointLightObject = new GameObject(lightName);
                Renderer renderer = child.GetChild(0).GetComponent<Renderer>(); // Only the renderer (and not the transform) has the 'true' position of 
                Light pointLight = pointLightObject.AddComponent<Light>();
                pointLight.type = LightType.Point;
                pointLight.color = color;
                pointLight.intensity = 10000f;
                pointLight.range = 2;
                pointLightObject.transform.parent = child;
                pointLightObject.transform.rotation = child.rotation;
                pointLightObject.transform.position = renderer.bounds.center + child.TransformDirection(new Vector3(0,0.1f,0)); // light is offset to the front of the signal by 0.1 m
                break;
            }
        }
    }

    // Finds the corresponding light_on node for a given light
    Transform FindNode(Transform lightTransform, string nodeNameContains)
    {
        foreach (Transform sibling in lightTransform.parent)
        {
            if (sibling.name.Contains(nodeNameContains))
            {
                return sibling;
            }
        }
        return null;
    }


    // Turns respective point lights on and off depending on the input char armState
    void SwitchLights(Light redLight, Light greenLight, Light yellowLight, char armState, Transform redNode, Transform greenNode, Transform yellowNode)
    {
        if (armState == 'G' ||armState == 'g' )
        {
            redLight.enabled = false;
            greenLight.enabled = true;
            yellowLight.enabled = false;
            redNode.gameObject.SetActive(false);
            greenNode.gameObject.SetActive(true);
            yellowNode.gameObject.SetActive(false);

        }

        else if (armState == 'r')
        {
            redLight.enabled = true;
            greenLight.enabled = false;
            yellowLight.enabled = false;
            redNode.gameObject.SetActive(true);
            greenNode.gameObject.SetActive(false);
            yellowNode.gameObject.SetActive(false);

        }

        else if (armState == 'y')
        {
            redLight.enabled = false;
            greenLight.enabled = false;
            yellowLight.enabled = true;
            redNode.gameObject.SetActive(false);
            greenNode.gameObject.SetActive(false);
            yellowNode.gameObject.SetActive(true);
        }

        else
        {
            redLight.enabled = false;
            greenLight.enabled = false;
            yellowLight.enabled = false;
            redNode.gameObject.SetActive(false);
            greenNode.gameObject.SetActive(false);
            yellowNode.gameObject.SetActive(false);        
        }

    }
}

