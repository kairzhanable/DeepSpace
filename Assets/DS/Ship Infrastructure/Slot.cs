using UnityEngine;

namespace DeepSpace
{
    public class Slot : MonoBehaviour
    {
        public ModuleType moduleType {get;}
        public int moduleSize {get;}
        public Slot (ModuleType moduleType, int moduleSize)
        {
            this.moduleType = moduleType;
            this.moduleSize = moduleSize;
        }

        public bool IsAvailable(Module module)
        {
            if (this.moduleType != module.moduleType)
                return false;

            if (this.moduleSize < module.moduleSize)
                return false;
                
            return true;
        }
    }
}