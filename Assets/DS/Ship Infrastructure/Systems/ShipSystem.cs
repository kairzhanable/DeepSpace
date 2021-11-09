using UnityEngine;
using System;

namespace DeepSpace
{
    public abstract class ShipSystem : MonoBehaviour
    {
        public abstract bool AddModule(Module module);
        public abstract bool RemoveModule(Module module);

        public virtual Type GetSystemType()
        {
            return this.GetType();
        }
    }
}
