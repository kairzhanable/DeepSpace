using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace DeepSpace
{
    [CreateAssetMenu(fileName = "Boost", menuName = "DeepSpace/Buffs/Boost", order = 0)]  
    public class Boost : Buff
    {
        [SerializeField]
        private float duration;
        [SerializeField]
        private float multiplyer;

        public override void init(MonoBehaviour mono) 
        {
            this.mono = mono;
            this.pairs = new List<Pair>();
        }

        public override bool apply(IBuffableModule module)
        {
            Engine engine = module as Engine;
            if (engine == null)
                return false;

            this.AddMod( engine.engineComponent.maxTrust, new MultiplyModifire(multiplyer, this));
            mono.StartCoroutine(cancel(duration, module));
            return true;
        }
    }
}