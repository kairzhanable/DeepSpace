using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class DetachSystem : ShipSystem
{
    private SystemElements<IDetachable> detachables;
    public DetachSystem()
    {
        this.detachables = new SystemElements<IDetachable>();
    }
    public override bool AddModule(Module module)
    {
        bool res = false;
        res = res || detachables.Add(module);
        return res;
    }

    public override bool RemoveModule(Module module)
    {
        bool res = false;
        res = res || detachables.Remove(module);
        return res;
    }
}

public interface IDetachable
{
    bool Detach();
}