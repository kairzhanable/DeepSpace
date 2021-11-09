using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SpaceGraphicsToolkit;

namespace DeepSpace
{
    public class Engine : Module, IEngine, IBuffableModule
    {

        [SerializeField]
        private EngineStat statsData;
        private EngineComponent _engineComponent;

        private EngineEffect _effect;
        public EngineEffect effect { get { return _effect; } }

        public EngineComponent engineComponent { get { return _engineComponent; } }
        public float force_coefficient => _engineComponent.force_coefficient;
        public void ApplyForce(float force_coefficient, float throttle_coefficient) { _engineComponent.ApplyForce(force_coefficient, throttle_coefficient); }
        public void Recalculation() { _engineComponent.Recalculation(); }
        public void SetRigidbody(Rigidbody rigidbody) { _engineComponent.SetRigidbody(rigidbody); }
        public Vector3 possible_force(float coefficient) { return _engineComponent.possible_force(coefficient); }
        public Vector3 possible_torque(float coefficient) { return _engineComponent.possible_torque(coefficient); }

        void Awake()
        {
            this._effect = new EngineEffect(this);
            _engineComponent = new EngineComponent(this, statsData, effect);
            base.init(statsData.moduleType, statsData.moduleSize);
        }

        public void applyEffect(Color color){
            this.effect.ApplyEffect(color);
        }

        public void removeEffect(){
            this.effect.removeEffect();
        }

        private void FixedUpdate()
        {
            _engineComponent.FixedUpdate();
        }

        public void ApplyBuff(Buff buff)
        {
            buff.apply(this);
        }

        public void CeaseBuff(Buff buff)
        {
            buff.cease(this);
        }
    }


    public class EngineEffect
    {

        private List<Light> lights;
        private List<SpriteRenderer> sprites;
        private SgtThruster throttleController;
        private Color defultColor;

        public EngineEffect(MonoBehaviour initializator)
        {
            this.lights = new List<Light>();
            this.lights = initializator.gameObject.transform.GetComponentsInChildrenRecursively<Light>(this.lights);
            this.sprites = new List<SpriteRenderer>();
            this.sprites = initializator.gameObject.transform.GetComponentsInChildrenRecursively<SpriteRenderer>(this.sprites);
            this.throttleController = initializator.gameObject.GetComponentInChildren<SgtThruster>();
            defultColor = Color.green;
        }

        public void removeEffect(){
            Debug.Log("OFF");
            ApplyEffect(defultColor);
        }

        public void ApplyThrottle(float throttle)
        {
            throttleController.Throttle = throttle;
            foreach (var light in lights)
                light.intensity = throttle;
        }

        public void ApplyEffect(Color color)
        {
            foreach (var light in lights)
                light.color = color;
            foreach (var sprite in sprites)
                sprite.color = color;
        }
    }

    public class EngineComponent
    {

        private Rigidbody rigidbody;
        private Vector3 _force;
        private Vector3 _torque;
        private float _throttle;
        private Vector3 final_force;
        private EngineEffect effect;

        private float _force_coefficient;
        private BuffableStat _maxTrust;

        private GameObject gameObject;

        public BuffableStat maxTrust { get { return _maxTrust; } }
        public float force_coefficient { get { return _force_coefficient; } }
        public float throttle { get { return _throttle; } }

        public Vector3 possible_force(float coefficient) { return _force * coefficient; }
        public Vector3 possible_torque(float coefficient) { return _torque * coefficient; }

        public EngineComponent(MonoBehaviour initializator, EngineStat statsData, EngineEffect effect)
        {
            this.effect = effect;
            _maxTrust = new BuffableStat(statsData.maxTrust);
            this.gameObject = initializator.gameObject;

        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        private Vector3 old_force = Vector3.zero;
        private float alpha = 0.5f;

        public void ApplyForce(float force_coefficient, float throttle_coefficient)
        {
            this._force_coefficient = force_coefficient;
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position;
            Vector3 force = -direction * (_maxTrust.buffedValue * _force_coefficient);
            effect.ApplyThrottle(Mathf.Clamp01(throttle_coefficient * (_maxTrust.buffedValue / _maxTrust.defultValue)));

            final_force = old_force * (1-alpha) + force * alpha;

            old_force = final_force;
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - rigidbody.transform.position + rigidbody.transform.rotation * rigidbody.centerOfMass;
            _force = -direction * _maxTrust.buffedValue;
            _torque = Vector3.Cross(position, -direction * _maxTrust.buffedValue);
        }

        public void FixedUpdate()
        {
            rigidbody.AddForceAtPosition(final_force, gameObject.transform.position);
        }
    }
}