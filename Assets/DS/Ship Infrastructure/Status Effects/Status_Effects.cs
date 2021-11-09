using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace DeepSpace
{







    public interface IBuffableModule
    {
        void ApplyBuff(Buff buff);
        void CeaseBuff(Buff buff);
    }









    public abstract class Buff : ScriptableObject
    {
        protected IEnumerator cancel(float duration, Module module)
        {
            yield return new WaitForSeconds(duration);
            this.cease(module);
        }

        protected MonoBehaviour mono;
        protected List<Modifire> mods;


        public abstract void init(MonoBehaviour mono);
        public abstract void apply(Module module);
        public abstract void cease(Module module);
    }








    public class MultiplyModifire : Modifire
    {
        public MultiplyModifire(float value, Type type_buff)
        {
            this._type_buff = type_buff;
            this.value = value;
        }


        private float value;
        private Type _type_buff;
        public override Type type_buff { get { return  _type_buff; } }

        public override float getEffect(float defult)
        {
            return defult * value;
        }
    }







    public abstract class Modifire
    {
        public abstract Type type_buff{ get; }
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
            Debug.Log(_buffedValue);
            return _buffedValue;
        }

        public Modifire ApplyMod(Modifire modifire)
        {
            if(contain(modifire)){
                Debug.Log("дубликат");
                return modifire;
            }
            refresh = true;
            _modifires.Add(modifire);
            return modifire;
        }

        private bool contain(Modifire modifire){
            Type type = modifire.type_buff;
            foreach(var mod in _modifires){
                if (mod.type_buff == type){
                    return true;
                }
            }
            return false;
        }

        public void CeaseMod(Modifire modifire)
        {
            if(!contain(modifire)){
                Debug.Log("не найден");
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