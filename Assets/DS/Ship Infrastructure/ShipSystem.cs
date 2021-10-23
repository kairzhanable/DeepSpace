using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public abstract class ShipSystem
{
    public abstract bool AddModule(Module module);
    public abstract bool RemoveModule(Module module);
}

public class SystemElements<T>
{
    private List<T> elements {get; set;}
    public SystemElements()
    {
        this.elements = new List<T>();
    }

    public bool Add(Module module)
    {
        if(module is T)
        {
            this.elements.Add((T)(object)module);
            return true;
        }
        return false;
    }

    public bool Remove(Module module)
    {
        if(module is T)
        {
            this.elements.Remove((T)(object)module);
            return true;
        }
        return false;
    }
}