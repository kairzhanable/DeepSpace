using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace{
    public abstract class ShipEvent 
    {
        public abstract void Apply(ShipSystem system);
        public abstract void Apply(Module module);
    }
}

