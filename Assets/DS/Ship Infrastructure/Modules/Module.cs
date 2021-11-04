using UnityEngine;

namespace DeepSpace
{
    public abstract class Module : MonoBehaviour
    {
        private ModuleType _moduleType;
        public ModuleType moduleType { get { return _moduleType; } }
        private int _moduleSize;
        public int moduleSize { get { return _moduleSize; } }
        private bool _detachable;
        public bool detachable { get { return _detachable; } }
        private ShipEcosystem _shipEcosystem;
        public ShipEcosystem shipEcosystem { get { return _shipEcosystem; } set { _shipEcosystem = value; } }

        public void init(ModuleType moduleType, int moduleSize)
        {
            this._moduleType = moduleType;
            this._moduleSize = moduleSize;
        }
    }
}

