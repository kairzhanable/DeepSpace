using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class EnergySystem : ShipSystem
{
    private SystemElements<IEConsumer> e_consumers;
    private SystemElements<IEGenerator> e_generators;
    private SystemElements<IEStorage> e_storages;
    
    public EnergySystem()
    {
        this.e_consumers = new SystemElements<IEConsumer>();
        this.e_generators = new SystemElements<IEGenerator>();
        this.e_storages = new SystemElements<IEStorage>();
    }
    public override bool AddModule(Module module)
    {
        bool res = false;
        if(e_consumers.Add(module))
        {
            res = true;
            Connect(module);
        }
        if (e_generators.Add(module))
        {
            res = true;
            Connect(module);
        }
        res = res || e_storages.Add(module);
        return res;
    }

    private void Connect(Module module)
    {
        if(!(module is IActiveEElement))
            return;
        IActiveEElement element = (IActiveEElement)module;
        element.ConnectToESystem(this);
    }

    public override bool RemoveModule(Module module)
    {
        bool res = false;
        res = res || e_consumers.Remove(module);
        res = res || e_generators.Remove(module);
        res = res || e_storages.Remove(module);
        return res;
    }
}

public interface IActiveEElement
{
    void ConnectToESystem(EnergySystem system);
}

public interface IEConsumer : IActiveEElement
{
    bool TakeEnergy(float amount);
}

public interface IEGenerator : IActiveEElement
{
    bool GenerateEnergy(float amount);
}

public interface IEStorage
{
    float maxCapacity {get;}
    float currentCapacity {get; set;}
}