using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace DeepSpace
{
    [CreateAssetMenu(fileName = "TMPBuff", menuName = "DeepSpace/Buffs/TMPBuff", order = 0)]  
    public class TMPBuff : Buff
    {
        [SerializeField]
        private float duration;
        [SerializeField]
        private float multiplyer;

        public override void init(MonoBehaviour mono)
        {
            this.mono = mono;
            this.mods = new List<Modifire>();
           
        }

        public override void apply(Module module)
        {
            IEngine engine = module as IEngine;
            if (engine != null)
            {
                BuffableStat stat = engine.engineComponent.maxTrust;
                MultiplyModifire mod = new MultiplyModifire(multiplyer, this.GetType());
                mods.Add(stat.ApplyMod(mod));
                engine.applyEffect(Color.red);
            }                
            
            mono.StartCoroutine(cancel(duration, module));
        }

        public override void cease(Module module)
        {
            IEngine engine = module as IEngine;
            if (engine == null)
                return;
            engine.removeEffect();
            foreach(var mod in mods){
                engine.engineComponent.maxTrust.CeaseMod(mod);
            }
            
        }
    }
}