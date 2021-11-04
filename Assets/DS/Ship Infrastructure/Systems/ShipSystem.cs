using UnityEngine;
using System;

namespace DeepSpace
{
    public abstract class ShipSystem : MonoBehaviour
    {
        public abstract bool AddModule(Module module);
        public abstract bool RemoveModule(Module module);
        public SendEvent sendEvent;

        public virtual Type GetSystemType()
        {
            return this.GetType();
        }

        public virtual void ApplyEvent(ShipEvent _event)
        {
            _event.Apply(this);
        }
    }

    public delegate void SendEvent(ShipEvent _event);
}
