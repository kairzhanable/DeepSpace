using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace{
    public class Engine : Module, IEngine    
    {
        private Vector3 _force;
        private Vector3 _torque;
        private Rigidbody rigidbody;

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

        public Rigidbody Rigidbody { get => rigidbody; set => rigidbody = value; }

        public void ApplyForce(float coefficient)
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position;
            Vector3 force = -direction * (statsData.maxTrust * coefficient);
            Rigidbody.AddForceAtPosition(force, position);
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - Rigidbody.transform.position + Rigidbody.transform.rotation * Rigidbody.centerOfMass;
            _force = -direction * statsData.maxTrust;
            _torque = Vector3.Cross(position, -direction * statsData.maxTrust);
        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.Rigidbody = rigidbody;
        }

        void Start()
        {
            base.init(statsData.moduleType, statsData.moduleSize);
        }

        void Update()
        {
            
        }
    }
}
