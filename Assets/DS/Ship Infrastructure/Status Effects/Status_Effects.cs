using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace DeepSpace
{


    public interface IBuffableModule
    {
        List<Buff> buffs { get; }

        void ApplyBuff(Buff buff);
        void CeaseBuff(Buff buff);
    }



    public abstract class Buff : ScriptableObject
    {

        protected IEnumerator cancel(float duration, IBuffableModule module)
        {
            yield return new WaitForSeconds(duration);
            this.cease();
            module.CeaseBuff(this);
        }

        protected MonoBehaviour mono;
        protected List<Pair> pairs;
        protected void AddMod(BuffableStat stat, Modifire mod){
            pairs.Add(new Pair( stat, stat.ApplyMod(mod)));
        }

        public abstract void init(MonoBehaviour mono);
        public abstract bool apply(IBuffableModule module);
        
        public void cease(){
            foreach(var pair in pairs){
                pair.stat.CeaseMod(pair.mod);
            }
        }
    }

    public class Pair{
        public BuffableStat stat;
        public Modifire mod;
        public Pair(BuffableStat stat, Modifire mod){
            this.stat = stat;
            this.mod = mod;
        }
    }



    public class MultiplyModifire : Modifire
    {
        public MultiplyModifire(float value, Buff buff)
        {
            this._buff = buff;
            this.value = value;
        }


        private float value;
        private Buff _buff;
        public override Buff buff { get { return  _buff; } }

        public override float getEffect(float defult)
        {
            return defult * value;
        }
    }



    public abstract class Modifire
    {
        public abstract Buff buff{ get; }
        public abstract float getEffect(float defult);
    }


    public class BuffableStat
    {
        public BuffableStat(float defultValue)
        {
            _defultValue = defultValue;
            _modifires = new List<Modifire>();
            refresh = true;
        }

        private List<Modifire> _modifires;
        private float _defultValue;
        private float _buffedValue;
        private bool refresh;
        public float defultValue { get { return _defultValue; } }
        public float buffedValue { get { return calcBuffedValue(); } }

        private float calcBuffedValue()
        {
            if (refresh)
            {
                _buffedValue = _defultValue;
                foreach (var mod in _modifires)
                {
                    _buffedValue += mod.getEffect(_defultValue);
                }
                refresh = false;
            }
            return _buffedValue;
        }

        public Modifire ApplyMod(Modifire modifire)
        {
            if(contain(modifire)){
                return modifire;
            }
            refresh = true;
            _modifires.Add(modifire);
            return modifire;
        }

        private bool contain(Modifire modifire){
            Buff buff = modifire.buff;
            foreach(var mod in _modifires){
                if (mod.buff == buff){
                    return true;
                }
            }
            return false;
        }

        public void CeaseMod(Modifire modifire)
        {
            if(!contain(modifire)){
                return;
            }
            refresh = true;
            _modifires.Remove(modifire);
        }
    }
}




/*public void CeaseBuff<T>()
{
    foreach (var modifire in _modifires)
    {
        if (modifire is T)
        {
            CeaseBuff(buff);
        }
    }
}*/


/*public class DEBUGSTATUSEFECT : StatusEffect
{

    private float duration;
    private float multiplier;

    private float buffedDelta;
    private IEnumerator coroutine;

    public void init(float duration, float multiplier){
        this.multiplier = multiplier;
        this.duration = duration;
    }

    public override void Apply(Module module)
    {
        if(module is IEngine){

            IEngine engine = (IEngine)module;
            EngineComponent engineComponent = engine.engineComponent;

            Debug.Log(engineComponent.maxTrust.buffedValue);

            buffedDelta = multiplier * engineComponent.maxTrust.defultValue;
            engineComponent.maxTrust.AddBuffStatus(buffedDelta);

            coroutine = this.waitingForCompletion(engineComponent);
            StartCoroutine(coroutine);
        }
    }

    private IEnumerator waitingForCompletion(EngineComponent engineComponent){
        yield return new WaitForSeconds(duration);
        end(engineComponent);
        Debug.Log(engineComponent.maxTrust.buffedValue);
    }

    private void end(EngineComponent engineComponent){
        engineComponent.maxTrust.AddBuffStatus(-buffedDelta);
    }
}*/