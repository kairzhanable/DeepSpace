using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace
{
    public class ShipEcosystem : MonoBehaviour
    {
        private List<ShipSystem> systems;
        private List<Slot> engineSlots;
        private List<Slot> externalModuleSlots;
        private List<Slot> internalModuleSlots;

        void Awake()
        {
            systems = new List<ShipSystem>(gameObject.GetComponents<ShipSystem>());
            Slot[] allSlots = gameObject.GetComponentsInChildren<Slot>();

            engineSlots = new List<Slot>();
            externalModuleSlots = new List<Slot>();
            internalModuleSlots = new List<Slot>();

            foreach(ShipSystem system in systems){
                system.sendEvent = SendEvent;
            }

            foreach(Slot slot in allSlots){
                switch(slot.moduleType){
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
        }

        public void SendEvent(ShipEvent _event){
            foreach(ShipSystem system in systems){
                system.ApplyEvent(_event);
            }
        }

        void Update()
        {
            
        }

        public void AddModule(Module module, Slot slot) 
        {
            if(!slot.InstallModule(module))
                return;
            foreach(ShipSystem sys in systems)
            {
                sys.AddModule(module);
            }
        }

        public void AddModule(GameObject module, Slot _slot){
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
            if(!module.detachable)    
                return;         
            foreach(ShipSystem sys in systems)
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