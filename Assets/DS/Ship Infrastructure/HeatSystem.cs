using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class HeatSystem : ShipSystem
{
    private SystemElements<IHConsumer> h_consumers;
    private SystemElements<IHGenerator> h_generators;
    private SystemElements<IHStorage> h_storages;

    public HeatSystem()
    {
        this.h_consumers = new SystemElements<IHConsumer>();
        this.h_generators = new SystemElements<IHGenerator>();
        this.h_storages = new SystemElements<IHStorage>();
    }
    
    public override bool AddModule(Module module)
    {
        bool res = false;
        if(h_consumers.Add(module))
        {
            res = true;
            Connect(module);
        }
        if (h_generators.Add(module))
        {
            res = true;
            Connect(module);
        }
        res = res || h_storages.Add(module);
        return res;
    }

    private void Connect(Module module)
    {
        if(!(module is IActiveHElement))
            return;
        IActiveHElement element = (IActiveHElement)module;
        element.ConnectToHSystem(this);
    }

    public override bool RemoveModule(Module module)
    {
        bool res = false;
        res = res || h_consumers.Remove(module);
        res = res || h_generators.Remove(module);
        res = res || h_storages.Remove(module);
        return res;
    }
}

public interface IActiveHElement
{
    void ConnectToHSystem(HeatSystem system);
}

public interface IHConsumer : IActiveHElement
{
    bool TakeHeat(float amount);
}

public interface IHGenerator : IActiveHElement
{
    bool GenerateHeat(float amount);
}

public interface IHStorage
{
    float maxCapacity {get;}
    float currentCapacity {get; set;}
}