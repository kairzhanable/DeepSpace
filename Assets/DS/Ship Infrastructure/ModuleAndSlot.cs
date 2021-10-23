using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

namespace DeepSpace
{
    public class Module
    {
        public ModuleType moduleType {get;}
        public int moduleSize {get;}

        public Module(ModuleType moduleType, int moduleSize)
        {
            this.moduleType = moduleType;
            this.moduleSize = moduleSize;
        }
    }
}

public enum ModuleType 
{
    EXTERNAL,
    INTERNAL
}

public class Slot 
{
    public ModuleType moduleType {get;}
    public int moduleSize {get;}
    public Slot (ModuleType moduleType, int moduleSize)
    {
        this.moduleType = moduleType;
        this.moduleSize = moduleSize;
    }

    public bool IsAvailable(Module module)
    {
        if (this.moduleType != module.moduleType)
            return false;

        if (this.moduleSize < module.moduleSize)
            return false;
            
        return true;
    }
}