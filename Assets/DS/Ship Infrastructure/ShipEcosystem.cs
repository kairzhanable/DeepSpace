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

        void Start()
        {
            systems = new List<ShipSystem>();
            Rigidbody rb = gameObject.GetComponent<Rigidbody>();

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
        }

        void Update()
        {
            
        }

        void AddModule(Module module, Slot slot) 
        {
            foreach(ShipSystem sys in systems)
            {
                sys.AddModule(module);
            }
        }

        void RemoveModule(Module module)
        {
            foreach(ShipSystem sys in systems)
            {
                sys.RemoveModule(module);
            }
        }
    }
}