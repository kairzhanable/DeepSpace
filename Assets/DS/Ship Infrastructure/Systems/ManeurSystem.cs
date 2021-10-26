using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using System.Collections.Generic;
using System;

namespace DeepSpace
{
    public class ManeurSystem : ShipSystem
    {
        private SystemElements<IEngine> IEngins;
        private Rigidbody rigidbody;
        private RCS rcs;
        private VPID vPID;
        private QPID qPID;

        void Awake() {
            rigidbody = gameObject.GetComponent<Rigidbody>();
            rcs = new RCS(rigidbody);
            qPID = new QPID(rigidbody);
            vPID = new VPID(rigidbody);
            
            IEngins = new SystemElements<IEngine>();
        }

        public void ApplyManeur(Vector3 desired_speed, Quaternion desiredRotation){
            rcs.Refresh(IEngins);
            Vector3 desiredForce = vPID.calcForce(desired_speed);
            Vector3 desiredTorque = qPID.calcTorque(desiredRotation);
            //rigidbody.AddRelativeForce(desiredForce);
            //rigidbody.AddRelativeTorque(desiredTorque);
            rcs.Update(desiredForce, desiredTorque * 1000f);
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
            if(!(module is IPhysical))
                return;
            IPhysical element = (IPhysical)module;
            element.SetRigidbody(rigidbody);
        }
    }

    public interface IControlled{

    }

    public interface IPhysical{
        void SetRigidbody(Rigidbody rigidbody);
    }

    public interface IEngine:IPhysical{
        void ApplyForce(float force_coefficient, float throttle_coefficient);
        void Recalculation();

        Vector3 force {get;}
        Vector3 torque {get;}
    }

    public class RCS{
        private Vector<double> initialGuess; // вектор начальных значений тяги ([0] * кол-во двиг.)
        private Vector3[] forces;            // вектор значений сил
        private Vector3[] torques;           // вектор значений моментов
        private Rigidbody rigidbody;
        private SystemElements<IEngine> IEngins;
        private Vector3 desiredForce;
        private Vector3 desiredTorque;

        public RCS(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        public void Refresh(SystemElements<IEngine> IEngins){
            this.IEngins = IEngins;
            forces = new Vector3[IEngins.Length];
            torques = new Vector3[IEngins.Length];
            initialGuess = Vector<double>.Build.Dense(IEngins.Length);
        }

        private Vector<double> OfFunction(Func<Vector<double>, double> function, Vector<double> initialGuess, double tolerance = 1e-100, int maxIterations = 100000)
        {
            var objective = ObjectiveFunction.Value(function);
            var result = NelderMeadSimplex.Minimum(objective, initialGuess, tolerance, maxIterations);
            return result.MinimizingPoint;
        }

        private double calculate(Vector<double> coeff)
        {
            Vector3 totalForces = new Vector3(0, 0, 0);
            Vector3 totalTorques = new Vector3(0, 0, 0);
            double d = 0;                                                                       // фактор гашения
            for (int trusterId = 0; trusterId < IEngins.Length; trusterId++)
            {
                totalForces += forces[trusterId] * (float)coeff[trusterId];
                totalTorques += torques[trusterId] * (float)coeff[trusterId];
                if (coeff[trusterId] > 1 || coeff[trusterId] < 0)
                {
                    d += 999 + coeff[trusterId] * 999;
                }
            }
            Vector3 errorLinear = (totalForces - rigidbody.transform.rotation * desiredForce);
            Vector3 errorAngle = (totalTorques - rigidbody.transform.rotation * desiredTorque);
            return (double)(errorLinear.x * errorLinear.x +
                            errorLinear.y * errorLinear.y +
                            errorLinear.z * errorLinear.z +
                            errorAngle.x * errorAngle.x +
                            errorAngle.y * errorAngle.y +
                            errorAngle.z * errorAngle.z + d);
        }

        public void Update(Vector3 desiredForce, Vector3 desiredTorque)
        {
            this.desiredForce = desiredForce;
            this.desiredTorque = desiredTorque;
            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                engine.Recalculation();

                this.forces[i] = engine.force;
                this.torques[i] = engine.torque;
                
            }
            Func<Vector<double>, double> f_delegate = calculate;
            Vector<double> results = OfFunction(f_delegate, initialGuess);
            float max = 0;
            foreach(float result in results){
                if(result > max){
                    max = result;
                }
            }


            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                double result = results[i];
                engine.ApplyForce((float)result, (float)result / max);
            }
        }
    }

    public class QPID{

        private Rigidbody rigidbody;

        public QPID(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        private double old_angle = 0;
        private double old_speed = 0;
        private double old_torque = 0;
        private double ema_k = 0;

        public double alpfa = 0.001f;
        public double langle = 180f;
        public double soft = 0.1;

        public Vector3 calcTorque(Quaternion targetRotation){
            Quaternion current_rotation = rigidbody.transform.rotation;
            Quaternion diff = targetRotation * Quaternion.Inverse(current_rotation);
            Vector3 angularVelocity = rigidbody.angularVelocity;
            Vector3 local_diff_torque = new Vector3(diff.x, diff.y, diff.z);
            local_diff_torque = Vector3.Normalize(local_diff_torque);
            double angle = Quaternion.Angle(targetRotation, current_rotation);
            double speed = (old_angle - angle) / Time.deltaTime;
            double acceleration = (old_speed - speed) / Time.deltaTime;
            old_speed = speed;
            old_angle = angle;

            double time = angle / Mathf.Abs((float)speed);
            if(speed/acceleration > time * 0.8){
                ema_k = ema_k * (1-alpfa) + alpfa;

            } else {
                ema_k = ema_k * (1-alpfa) - alpfa;
            }
            if(angle < langle && angularVelocity.sqrMagnitude != 0f){
                //local_diff_torque += (angularVelocity * (float)(langle-angle)/(float)langle) / (float)soft;
            }
            if(angle < 1 && speed < 0.1){
                rigidbody.angularVelocity = new Vector3(0.0001f,0.0001f,0.0001f);
                return Vector3.zero;
            }
            return Quaternion.Inverse(rigidbody.transform.rotation) * local_diff_torque * (float)ema_k;
        }
    }

    public class VPID{
        private Rigidbody rigidbody;

        public VPID(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        public Vector3 calcForce(Vector3 desired_speed){
            return Vector3.Normalize(desired_speed - rigidbody.transform.InverseTransformDirection(rigidbody.velocity));
        }
    }

}