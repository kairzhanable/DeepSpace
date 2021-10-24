using UnityEngine;

namespace DeepSpace
{
    public abstract class Module : MonoBehaviour
    {
        private ModuleType _moduleType;
        public ModuleType moduleType {get{ return _moduleType;}}
        private int _moduleSize;
        public int moduleSize {get{ return _moduleSize;}}

        public void init(ModuleType moduleType, int moduleSize)
        {
            this._moduleType = moduleType;
            this._moduleSize = moduleSize;
        }
    }
}

