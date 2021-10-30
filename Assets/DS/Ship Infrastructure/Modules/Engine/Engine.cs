using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SpaceGraphicsToolkit;

namespace DeepSpace{
    public class Engine : Module, IEngine    
    {
        private Vector3 _force;
        private Vector3 _torque;

        private float _limit = 1;
        private float _throttle;

        private Rigidbody rigidbody;
        private SgtThruster throttleController;
        private float _force_coefficient;

        public EngineStat statsData;

        public Vector3    max_force             {get{return _force;}}
        public Vector3    max_torque             {get{return _torque;}}

        public Vector3    currentForce     {get{return _force * _force_coefficient;}}
        public Vector3    currentTorque     {get{return _torque * _force_coefficient;}}

        public float      limit             {get{return _limit;}        set{_limit = value;}}
        public float      throttle          {get{return _throttle; }    set{_throttle = value; throttleController.Throttle = value; }}
        public float      force_coefficient {get{return _force_coefficient;}}

        public Vector3 possible_force(float coefficient){return  _force * coefficient;}
        public Vector3 possible_torque(float coefficient){return  _torque * coefficient;}

        private Vector3 final_force;

        public void ApplyForce(float force_coefficient, float throttle_coefficient)
        {
            this._force_coefficient = force_coefficient;
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position;
            Vector3 force = -direction * (statsData.maxTrust * _force_coefficient * _limit);
            throttle = throttle_coefficient * _limit;
            final_force = force;
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - rigidbody.transform.position + rigidbody.transform.rotation * rigidbody.centerOfMass;
            _force = -direction * statsData.maxTrust;
            _torque = Vector3.Cross(position, -direction * statsData.maxTrust);
        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        void Awake()
        {
            base.init(statsData.moduleType, statsData.moduleSize);
            throttleController = gameObject.GetComponentInChildren<SgtThruster>();
        }

        void Update()
        {
            
        }

        public void FixedUpdate() {
            rigidbody.AddForceAtPosition(final_force, gameObject.transform.position);
        }
    }
}