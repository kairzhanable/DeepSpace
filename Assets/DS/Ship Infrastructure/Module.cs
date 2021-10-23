using UnityEngine;

namespace DeepSpace
{
    public class Module : MonoBehaviour
    {
        public ModuleType moduleType {get;}
        public int moduleSize {get;}

        public Module(ModuleType moduleType, int moduleSize)
        {
            this.moduleType = moduleType;
            this.moduleSize = moduleSize;
        }
    }
}

