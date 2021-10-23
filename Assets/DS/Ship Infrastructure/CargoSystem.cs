using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class CargoSystem : ShipSystem
{
    private SystemElements<ICargo> cargos;
    public CargoSystem()
    {
        this.cargos = new SystemElements<ICargo>();
    }
    public override bool AddModule(Module module)
    {
        bool res = false;
        res = res || cargos.Add(module);
        return res;
    }

    public override bool RemoveModule(Module module)
    {
        bool res = false;
        res = res || cargos.Remove(module);
        return res;
    }
}

public interface ICargo
{
    public int totalSlots {get;set;}
    public int freeSlots {get;set;}
    public int slotCapacity {get;set;}
}