using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace
{
    public class ShipEcosystem : MonoBehaviour
    {
        private List<ShipSystem> systems;

        private ArmorySystem armorySystem;
        private CargoSystem cargoSystem;
        private DetachSystem detachSystem;
        private EnergySystem energySystem;
        private HeatSystem heatSystem;
        private ManeurSystem maneurSystem;

        private List<Slot> engines;
        private List<Slot> externalModules;
        private List<Slot> internalModules;

        void Start()
        {
            systems = new List<ShipSystem>();
            Rigidbody rb = gameObject.GetComponent<Rigidbody>();
            engines = new List<Slot>();
            externalModules = new List<Slot>();
            internalModules = new List<Slot>();

            armorySystem = new ArmorySystem();
            cargoSystem = new CargoSystem();
            detachSystem = new DetachSystem();
            energySystem = new EnergySystem();
            heatSystem = new HeatSystem();
            maneurSystem = new ManeurSystem(rb);

            systems.Add(armorySystem);
            systems.Add(cargoSystem);
            systems.Add(detachSystem);
            systems.Add(energySystem);
            systems.Add(heatSystem);
            systems.Add(maneurSystem);

            Slot[] allSlots = gameObject.GetComponentsInChildren<Slot>();
            foreach(Slot slot in allSlots){
                switch(slot.moduleType){
                    case ModuleType.ENGINE:
                        engines.Add(slot);
                        break;
                    case ModuleType.EXTERNAL:
                        externalModules.Add(slot);
                        break;
                    case ModuleType.INTERNAL:
                        internalModules.Add(slot);
                        break;
                    default:
                        Debug.LogError("Неизвестный модуль.");
                        break;
                }
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

        public void AddModule(GameObject module, GameObject slot) 
        {
            Module _module = module.GetComponent<Module>();
            Slot _slot = slot.GetComponent<Slot>();
            AddModule(_module, _slot);
        }

        public void RemoveModule(Module module)
        {
            if(!detachSystem.RemoveModule(module))
                return;                                                 // если модуль не удаляемый, мы его не удаляем. логично.
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