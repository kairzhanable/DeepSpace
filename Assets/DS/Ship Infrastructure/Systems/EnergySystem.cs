using System;

namespace DeepSpace
{
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
            Connect(module);
            res = res || e_consumers.Add(module);
            res = res || e_generators.Add(module);
            res = res || e_storages.Add(module);
            return res;
        }

        private void Connect(Module module)
        {
            if (!(module is IActiveEElement))
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
        float maxCapacity { get; }
        float currentCapacity { get; set; }
    }
}