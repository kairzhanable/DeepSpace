using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SpaceGraphicsToolkit;

namespace DeepSpace{
    public class Engine : Module, IEngine    
    {
        private Vector3 _force;
        private Vector3 _torque;
        private float _throttle;
        private Rigidbody rigidbody;
        private SgtThruster throttleController;

        public EngineStat statsData;
        

        public Vector3 force 
        {
            get {return _force;}
            set {}
        }

        public Vector3 torque 
        {
            get {return _torque;}
            set {}
        }

        public float throttle{
            get { return _throttle; }
            set { _throttle = value; throttleController.Throttle = value; }
        }

        public Rigidbody Rigidbody { get => rigidbody; set => rigidbody = value; }

        public void ApplyForce(float force_coefficient, float throttle_coefficient)
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position;
            Vector3 force = -direction * (statsData.maxTrust * force_coefficient);
            throttle = force_coefficient;
            Rigidbody.AddForceAtPosition(force, position);
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - Rigidbody.transform.position + Rigidbody.transform.rotation * Rigidbody.centerOfMass;
            _force = -direction * statsData.maxTrust;
            _torque = Vector3.Cross(position, -direction * statsData.maxTrust);  //TODO учесть массу и момент инерции
        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.Rigidbody = rigidbody;
        }

        void Awake()
        {
            base.init(statsData.moduleType, statsData.moduleSize);
            throttleController = gameObject.GetComponentInChildren<SgtThruster>();
        }

        void Update()
        {
            
        }
    }
}
