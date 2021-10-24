using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using System;

namespace DeepSpace
{
    public class ManeurSystem : ShipSystem
    {
        private SystemElements<IEngine> IEngins;
        private Rigidbody rigidbody;
        private RCS rcs;

        public ManeurSystem(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
            rcs = new RCS(rigidbody);
            IEngins = new SystemElements<IEngine>();
        }

        public void ApplyManeur(Vector3 desiredForce, Vector3 desiredTorque){
            rcs.Refresh(IEngins);
            rcs.Update(desiredForce, desiredTorque);
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

    public interface IPhysical{
        void SetRigidbody(Rigidbody rigidbody);
    }

    public interface IEngine:IPhysical{
        void ApplyForce(float coefficient);
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
                forces[i] = engine.force;
                torques[i] = engine.torque;
            }

            Func<Vector<double>, double> f_delegate = calculate;
            Vector<double> results = OfFunction(f_delegate, initialGuess);

            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                double result = results[i];
                engine.ApplyForce((float)result);
            }
        }
    }

}