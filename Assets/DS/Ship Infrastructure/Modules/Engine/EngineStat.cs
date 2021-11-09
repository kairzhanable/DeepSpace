using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace
{
    [CreateAssetMenu(fileName = "EngineStat", menuName = "DeepSpace/Stats/EngineStat", order = 0)]
    public class EngineStat : ScriptableObject
    {
        public ModuleType moduleType;
        public int moduleSize;
        public float maxTrust;
    }
}
