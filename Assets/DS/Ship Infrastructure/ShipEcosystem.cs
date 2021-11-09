using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace DeepSpace
{
    public class ShipEcosystem : MonoBehaviour
    {
        private Dictionary<Type, ShipSystem> systems;
        private List<IBuffableModule> buffableModules;

        private Dictionary<Module, List<Type>> moduleI;

        void Awake()
        {
            buffableModules = new List<IBuffableModule>();
            systems = new Dictionary<Type, ShipSystem>();
            moduleI = new Dictionary<Module, List<Type>>();
            addSystems(new List<ShipSystem>(gameObject.GetComponents<ShipSystem>()));
        }

        public T GetSystem<T>() where T : ShipSystem
        {
            var systemType = typeof(T);
            return (T)systems[systemType];
        }

        private void addSystems(List<ShipSystem> systems)
        {
            foreach (var system in systems)
            {
                addSystem(system);
            }
        }

        private void addSystem(ShipSystem system)
        {
            Type systemType = system.GetSystemType();
            this.systems[systemType] = system;
        }


        public void sendBuff(Buff buff)
        {
            buff.init(this);
            foreach (var module in buffableModules)
            {
                /*List<Type> a = moduleI[(Module)module];
                List<Type> b = buff.targetTypes;
                if(a.Any(b.Contains)){*/
                module.ApplyBuff(buff);
                //}
            }
        }

        public void AddModule(Module module, Slot slot)
        {
            if (!slot.InstallModule(module))
                return;
            bool added = false;
            foreach (ShipSystem sys in systems.Values)
            {
                added = added || sys.AddModule(module);
            }
            if (added)
            {
                if (module is IBuffableModule)
                {
                    buffableModules.Add((IBuffableModule)module);
                }

                List<Type> types = new List<Type>(module.GetType().GetInterfaces());
                moduleI[module] = types;
                module.shipEcosystem = this;
            }
        }

        public void AddModule(GameObject module, Slot _slot)
        {
            Module _module = module.GetComponent<Module>();
            AddModule(_module, _slot);
        }

        public void AddModule(GameObject module, GameObject slot)
        {
            Module _module = module.GetComponent<Module>();
            Slot _slot = slot.GetComponent<Slot>();
            AddModule(_module, _slot);
        }

        public void RemoveModule(Module module)
        {
            if (!module.detachable)
                return;
            foreach (ShipSystem sys in systems.Values)
            {
                sys.RemoveModule(module);
            }
        }

        public void RemoveModule(GameObject module)
        {
            Module _module = module.GetComponent<Module>();
            RemoveModule(_module);
        }
    }
}