using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SpaceGraphicsToolkit;

namespace DeepSpace
{
    public class Engine : Module, IEngine
    {

        [SerializeField]
        private EngineStat statsData;

        private EngineComponent engineComponent;
        public float force_coefficient => engineComponent.force_coefficient;
        public void ApplyForce(float force_coefficient, float throttle_coefficient) { engineComponent.ApplyForce(force_coefficient, throttle_coefficient); }
        public void Recalculation() { engineComponent.Recalculation(); }
        public void SetRigidbody(Rigidbody rigidbody) { engineComponent.SetRigidbody(rigidbody); }
        public Vector3 possible_force(float coefficient) { return engineComponent.possible_force(coefficient); }
        public Vector3 possible_torque(float coefficient) { return engineComponent.possible_torque(coefficient); }

        void Awake()
        {
            engineComponent = new EngineComponent(this, statsData);
            base.init(statsData.moduleType, statsData.moduleSize);

        }

        private void Start()
        {

        }

        private void Update()
        {
            engineComponent.Update();
        }

        private void FixedUpdate()
        {
            engineComponent.FixedUpdate();
        }
    }


    public class EngineComponent
    {

        private float old_throtle = 0;
        private Rigidbody rigidbody;
        private Vector3 _force;
        private Vector3 _torque;
        private float _throttle;
        private Vector3 final_force;
        private SgtThruster throttleController;
        private float _force_coefficient;
        private EngineStat statsData;
        private GameObject gameObject;
        private List<Light> Lights;

        public float force_coefficient { get { return _force_coefficient; } }
        public float throttle { get { return _throttle; } }

        public Vector3 possible_force(float coefficient) { return _force * coefficient; }
        public Vector3 possible_torque(float coefficient) { return _torque * coefficient; }

        public EngineComponent(MonoBehaviour initializator, EngineStat statsData)
        {
            this.Lights = new List<Light>();
            this.Lights = initializator.gameObject.transform.GetComponentsInChildrenRecursively<Light>(this.Lights);
            this.statsData = statsData;
            this.gameObject = initializator.gameObject;
            this.throttleController = initializator.gameObject.GetComponentInChildren<SgtThruster>();
        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        public void ApplyForce(float force_coefficient, float throttle_coefficient)
        {
            this._force_coefficient = force_coefficient;
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position;
            Vector3 force = -direction * (statsData.maxTrust * _force_coefficient);
            _throttle = throttle_coefficient;
            throttleController.Throttle = throttle_coefficient;
            final_force = force;
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - rigidbody.transform.position + rigidbody.transform.rotation * rigidbody.centerOfMass;
            _force = -direction * statsData.maxTrust;
            _torque = Vector3.Cross(position, -direction * statsData.maxTrust);
        }

        public void Update()
        {
            float alpfa = 0.1f;
            old_throtle = old_throtle * (1 - alpfa) + _throttle * alpfa;
            foreach (var light in Lights)
                light.intensity = _throttle;
        }

        public void FixedUpdate()
        {
            rigidbody.AddForceAtPosition(final_force, gameObject.transform.position);
        }
    }
}