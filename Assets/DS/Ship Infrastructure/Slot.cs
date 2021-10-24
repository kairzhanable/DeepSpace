using UnityEngine;

namespace DeepSpace
{
    public class Slot : MonoBehaviour
    {
        public ModuleType moduleType;
        public int moduleSize;

        private bool _free;
        public bool free{get {return _free;}}

        private Module installedModule;

        public Slot (ModuleType moduleType, int moduleSize)
        {
            this.moduleType = moduleType;
            this.moduleSize = moduleSize;
        }

        public bool RemoveModule(){
            if ((_free))
                return false;
            _free = true;
            return true;            
        }

        public bool InstallModule(Module module){
            if(!IsAvailable(module))
                return false;
            installedModule = module;
            module.gameObject.transform.position = gameObject.transform.position;
            module.gameObject.transform.rotation = gameObject.transform.rotation;
            module.gameObject.transform.parent = gameObject.transform;
            _free = false;
            return true;
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