using UnityEngine;
using System.Collections.Generic;
using System;

using DeepSpace;

namespace DeepSpace
{
    public class ManeurSystem : ShipSystem
    {
        private SystemElements<IEngine> IEngins;
        private Rigidbody rigidbody;
        private MyRCS rcs;
        private VPID vPID;
        private QPID qPID;

        void Awake()
        {
            rigidbody = gameObject.GetComponent<Rigidbody>();
            rcs = new MyRCS(rigidbody);
            qPID = new QPID(rigidbody);
            vPID = new VPID(rigidbody);
            IEngins = new SystemElements<IEngine>();     
        }

        public void ApplyManeur(Vector3 desired_speed, Quaternion desiredRotation)
        {
            Vector3 desiredForce = vPID.calcForce(desired_speed);
            Vector3 desiredTorque = qPID.calcTorque(desiredRotation);

            if (desiredForce != Vector3.zero || desiredTorque != Vector3.zero)
            {
                rcs.SetDesired(desiredForce, desiredTorque);
                rcs.Apply(IEngins);
            }
            else
            {
                for (int i = 0; i < IEngins.Length; i++)
                {
                    IEngins.get(i).ApplyForce(0, 0);
                }
            }
        }

        public override bool AddModule(Module module)
        {
            bool res = false;
            res = res || IEngins.Add(module);
            Connect(module);
            return res;
        }

        public override bool RemoveModule(Module module)
        {
            bool res = false;
            res = res || IEngins.Remove(module);
            return res;
        }

        private void Connect(Module module)
        {
            if (!(module is IPhysical))
                return;
            IPhysical element = (IPhysical)module;
            element.SetRigidbody(rigidbody);
        }
    }

    public interface IPhysical
    {
        void SetRigidbody(Rigidbody rigidbody);
    }

    public interface IEngine : IPhysical
    {
        void ApplyForce(float force_coefficient, float throttle_coefficient);
        void Recalculation();

        EngineComponent engineComponent { get; }
        float force_coefficient { get; }

        Vector3 possible_force(float coefficient);
        Vector3 possible_torque(float coefficient);
    }

    public class QPID
    {

        private Rigidbody rigidbody;
        private float max_Velocity = 1;

        public QPID(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        private float old_angle = 0;
        private float old_angularSpeed = 0;

        public Vector3 calcTorque(Quaternion targetRotation)
        {
            Quaternion current_rotation = rigidbody.transform.rotation;
            Quaternion diff = targetRotation * Quaternion.Inverse(current_rotation);
            var angle = 2 * Math.Acos(diff.w);
            var axis = Vector3.zero;
            if (angle != 0)
                axis = (float)(1 / Math.Sin(angle / 2)) * new Vector3(diff.x, diff.y, diff.z);
            var local_diff_torque_norm = axis.normalized;

            float error_angle = Quaternion.Angle(targetRotation, current_rotation);
            float speed_k = (float)(1 - ((360 - angle) / 360));

            Vector3 desiredSpeed = local_diff_torque_norm * 1000 * speed_k;
            Vector3 actualSpeed = rigidbody.angularVelocity;
            Vector3 diffSpeed = desiredSpeed - actualSpeed;

            return (Quaternion.Inverse(rigidbody.transform.rotation) * diffSpeed * 100000 * Time.deltaTime);

        }
    }

    public class VPID
    {
        private Rigidbody rigidbody;

        public VPID(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        public Vector3 calcForce(Vector3 desired_speed)
        {
            if ((rigidbody.velocity - desired_speed).sqrMagnitude < 0.1)
            {
                rigidbody.velocity = desired_speed;
                return Vector3.zero;
            }

            Vector3 res = (desired_speed - rigidbody.transform.InverseTransformDirection(rigidbody.velocity));

            float max_force = 500;

            if(res.sqrMagnitude > max_force * max_force)
                res = res.normalized * max_force;

            return res;
        }
    }

    public class MyRCS
    {
        private Rigidbody rigidbody;
        private Vector3 desiredForce;
        private Vector3 desiredTorque;
        private SystemElements<IEngine> IEngins;
        private List<float> coefs;

        public MyRCS(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
            coefs = new List<float>();
        }

        public void SetDesired(Vector3 desiredForce, Vector3 desiredTorque)
        {
            this.desiredForce = desiredForce;
            this.desiredTorque = desiredTorque;
        }

        public void Apply(SystemElements<IEngine> IEngins)
        {
            this.IEngins = IEngins;
            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngins.get(i).Recalculation();
            }
            refresh();
            float step = 0.5f;
            for (int iteration = 0; iteration < 200; iteration++)
            {
                if (!optimize(step))
                {
                    step /= 2f;
                }
                if (CalcError() <= 1)
                {
                    break;
                }
            }
            initEngine();
        }

        private void initEngine()
        {
            float max = 1;
            float min = float.MaxValue;
            for (int i = 0; i < IEngins.Length; i++)
            {
                if(coefs[i] > max)
                    max = coefs[i];
                if(coefs[i] < min)
                    min = coefs[i];
            }
            if (min < 0)
                min = 0; 
            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                engine.ApplyForce(coefs[i], (coefs[i] - min) / max);
            }
        }

        private bool optimize(float step)
        {

            float init_error = CalcError();
            int[] dirs = new int[] { 1, -1 };

            float min_error = init_error;
            bool optimized = false;
            int opt_engine = -1;
            float delta = 0;

            for (int i = 0; i < IEngins.Length; i++)
            {
                foreach (int dir in dirs)
                {
                    float new_error = vary(i, dir * step);
                    if (new_error < min_error)
                    {
                        min_error = new_error;
                        opt_engine = i;
                        delta = dir * step;
                        optimized = true;
                    }
                }
            }

            if (optimized)
            {
                coefs[opt_engine] += delta;
            }
            return optimized;
        }

        private float vary(int EngineID, float step)
        {
            float cacheCoefficient = coefs[EngineID];/*IEngins.get(EngineID).force_coefficient;*/
            if (cacheCoefficient + step > 1 || cacheCoefficient + step < 0)
                return float.MaxValue;
            Vector3 newForce = IEngins.get(EngineID).possible_force(cacheCoefficient + step);
            Vector3 newTorque = IEngins.get(EngineID).possible_torque(cacheCoefficient + step);
            return CalcError(EngineID, newForce, newTorque);
        }

        private void refresh()
        {
            coefs.Clear();
            for (int i = 0; i < IEngins.Length; i++)
            {
                coefs.Add(0);
            }
        }

        private float CalcError()
        {
            return CalcError(-1, Vector3.zero, Vector3.zero);
        }

        private float CalcError(int EngineID, Vector3 newForce, Vector3 newTorque)
        {

            Vector3 errorLinear = Vector3.zero;
            Vector3 errorAngle = Vector3.zero;

            for (int i = 0; i < IEngins.Length; i++)
            {
                if (i != EngineID)
                {
                    errorLinear += IEngins.get(i).possible_force(coefs[i]);
                    errorAngle += IEngins.get(i).possible_torque(coefs[i]);
                }
                else
                {
                    errorLinear += newForce;
                    errorAngle += newTorque;
                }
            }

            errorLinear -= rigidbody.transform.rotation * desiredForce;
            errorAngle -= rigidbody.transform.rotation * desiredTorque;

            return (errorLinear.x * errorLinear.x +
                    errorLinear.y * errorLinear.y +
                    errorLinear.z * errorLinear.z +
                    errorAngle.x * errorAngle.x +
                    errorAngle.y * errorAngle.y +
                    errorAngle.z * errorAngle.z);
        }
    }
}