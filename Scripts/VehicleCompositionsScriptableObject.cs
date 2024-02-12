using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Vehicle Composition", menuName = "TumVT/VehicleCompositionScriptableObject", order = 1)]
public class VehicleCompositionsScriptableObject : ScriptableObject
{
    public List<GameObject> PassengerCars = new List<GameObject>();
    public List<GameObject> Trucks = new List<GameObject>();
    public List<GameObject> Bicycles = new List<GameObject>();
    public List<GameObject> Persons = new List<GameObject>();
    public List<GameObject> Busses = new List<GameObject>();
    public List<GameObject> Taxis = new List<GameObject>();
}
