using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class ShipEcosystem : MonoBehaviour
{
    private List<ShipSystem> systems;
    void Start()
    {
        systems.Add(new ArmorySystem());
        systems.Add(new CargoSystem());
        systems.Add(new DetachSystem());
        systems.Add(new EnergySystem());
        systems.Add(new HeatSystem());
        systems.Add(new ManeurSystem());
    }

    void Update()
    {
        
    }

    void AddModule(Module module, Slot slot) 
    {
        foreach(ShipSystem sys in systems)
        {
            sys.AddModule(module);
        }
    }

    void RemoveModule(Module module)
    {
        foreach(ShipSystem sys in systems)
        {
            sys.RemoveModule(module);
        }
    }
}
