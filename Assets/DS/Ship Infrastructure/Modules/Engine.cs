using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DeepSpace{
    public class Engine : Module, IEngine    
    {
        private Vector3 _force;
        private Vector3 _torque;
        private Rigidbody rigidbody;

        public float maxTrust = 1; //TODO ВЫНЕСТИ В ПАРАМЕТРЫ ИЗ СКРИПТ.ОБДЖ

        public Engine(ModuleType moduleType, int moduleSize) : base(moduleType, moduleSize)
        {

        }

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
            Vector3 force = -direction * (maxTrust * coefficient);
            Rigidbody.AddForceAtPosition(force, position);
        }

        public void Recalculation()
        {
            Vector3 direction = gameObject.transform.forward;
            Vector3 position = gameObject.transform.position - Rigidbody.transform.position + Rigidbody.transform.rotation * Rigidbody.centerOfMass;
            _force = -direction * maxTrust;
            _torque = Vector3.Cross(position, -direction * maxTrust);
        }

        public void SetRigidbody(Rigidbody rigidbody)
        {
            this.Rigidbody = rigidbody;
        }

        void Start()
        {
            
        }

        void Update()
        {
            
        }
    }
}
