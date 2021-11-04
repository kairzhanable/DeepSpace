using System.Collections.Generic;
using UnityEngine;
using System;

namespace DeepSpace
{
    public class ShipEcosystem : MonoBehaviour
    {
        private Dictionary<Type, ShipSystem> systems;

        private List<Slot> engineSlots;
        private List<Slot> externalModuleSlots;
        private List<Slot> internalModuleSlots;

        void Awake()
        {
            systems = new Dictionary<Type, ShipSystem>();

            engineSlots = new List<Slot>();
            externalModuleSlots = new List<Slot>();
            internalModuleSlots = new List<Slot>();

            addSystems(new List<ShipSystem>(gameObject.GetComponents<ShipSystem>()));
            addSlots(new List<Slot>(gameObject.GetComponentsInChildren<Slot>()));
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
            system.sendEvent = SendEvent;
            Type systemType = system.GetSystemType();
            this.systems[systemType] = system;
        }

        private void addSlots(List<Slot> allSlots)
        {
            foreach (Slot slot in allSlots)
            {
                addSlot(slot);
            }
        }

        private void addSlot(Slot slot)
        {
            switch (slot.moduleType)
            {
                case ModuleType.ENGINE:
                    engineSlots.Add(slot);
                    break;
                case ModuleType.EXTERNAL:
                    externalModuleSlots.Add(slot);
                    break;
                case ModuleType.INTERNAL:
                    internalModuleSlots.Add(slot);
                    break;
                default:
                    Debug.LogError("Неизвестный модуль.");
                    break;
            }
        }

        public void SendEvent(ShipEvent _event)
        {
            foreach (ShipSystem system in systems.Values)
            {
                system.ApplyEvent(_event);
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
                module.shipEcosystem = this;
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