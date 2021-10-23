using UnityEngine;

namespace DeepSpace
{
    public class ManeurSystem : ShipSystem
    {
        private SystemElements<IForceApply> IForceAppliers;
        private SystemElements<IForceRecalculate> IForceRecalculates;
        private Rigidbody rigidbody;

        public ManeurSystem(Rigidbody rb)
        {
            rigidbody = rb;
            IForceAppliers = new SystemElements<IForceApply>();
            IForceRecalculates = new SystemElements<IForceRecalculate>();
        }
        public override bool AddModule(Module module)
        {
            bool res = false;
            Connect(module);
            res = res || IForceAppliers.Add(module);
            res = res || IForceRecalculates.Add(module);
            return res;
        }

        public override bool RemoveModule(Module module)
        {
            bool res = false;
            res = res || IForceAppliers.Remove(module);
            res = res || IForceRecalculates.Remove(module);
            return res;
        }

        private void Connect(Module module)
        {
            if(!(module is IPhysical))
                return;
            IPhysical element = (IPhysical)module;
            element.SetRigidbody(rigidbody);
        }
    }

    public interface IPhysical{
        void SetRigidbody(Rigidbody rb);
    }

    public interface IForceApply:IPhysical{
        void ApplyForce(float coefficient);
    }

    public interface IForceRecalculate:IPhysical{
        void Recalculate();

        Vector3 force {get;}
        Vector3 torque {get;}
    }
}