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

        void Start() {
            rigidbody = gameObject.GetComponent<Rigidbody>();
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

    public interface IControlled{

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

    public class SAS{

        private Rigidbody rigidbody;
        private Vector3 desiredForce;
        private Vector3 desiredTorque;
        private PID pidX;
        private PID pidY;
        private PID pidZ;

        public SAS(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        public void Update(Vector3 desiredVelocity, Quaternion desiredDirection)
        {
            Vector3 currentVelocity = rigidbody.velocity;
            Quaternion currentDirection = rigidbody.gameObject.transform.rotation;

            //pitch 
            //roll 
            //yaw
            //axis

            /*
            Quaternion up = new Quaternion(-v,0,0,1);
            Quaternion down = new Quaternion(v,0,0,1);
            Quaternion left = new Quaternion(0,-v,0,1);
            Quaternion right = new Quaternion(0,v,0,1);
            Quaternion roll_left = new Quaternion(0,0,v,1);
            Quaternion roll_right = new Quaternion(0,0,-v,1);
            */

            Quaternion current = rigidbody.gameObject.transform.rotation;
            Quaternion diff = desiredDirection * Quaternion.Inverse(current);
            Vector3 local_diff_torque = new Vector3(diff.x, diff.y, diff.z);

            



            //gameObject.GetComponent<Rigidbody>().AddTorque(local_diff_torque, ForceMode.Impulse);

            desiredForce = (desiredVelocity - currentVelocity) * rigidbody.mass;
            //desiredTorque = 
        }

    }

    public class PID
    {
        private double Ts;                  // Sample period in seconds
        private double K;                   // Rollup parameter
        private double b0, b1, b2;          // Rollup parameters
        private double a0, a1, a2;          // Rollup parameters
        private double y0 = 0;              // Current output
        private double y1 = 0;              // Output one iteration old
        private double y2 = 0;              // Output two iterations old
        private double e0 = 0;              // Current error
        private double e1 = 0;              // Error one iteration old
        private double e2 = 0;              // Error two iterations old

        /// <summary>
        /// PID Constructor
        /// </summary>
        /// <param name="Kp">Proportional Gain</param>
        /// <param name="Ki">Integral Gain</param>
        /// <param name="Kd">Derivative Gain</param>
        /// <param name="N">Derivative Filter Coefficient</param>
        /// <param name="OutputUpperLimit">Controller Upper Output Limit</param>
        /// <param name="OutputLowerLimit">Controller Lower Output Limit</param>
        public PID(double Kp, double Ki, double Kd, double N, double OutputUpperLimit, double OutputLowerLimit)
        {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.N = N;
            this.OutputUpperLimit = OutputUpperLimit;
            this.OutputLowerLimit = OutputLowerLimit;
        }

        /// <summary>
        /// PID iterator, call this function every sample period to get the current controller output.
        /// setpoint and processValue should use the same units.
        /// </summary>
        /// <param name="setPoint">Current Desired Setpoint</param>
        /// <param name="processValue">Current Process Value</param>
        /// <param name="ts">Timespan Since Last Iteration, Use Default Sample Period for First Call</param>
        /// <returns>Current Controller Output</returns>
        public double PID_iterate(double setPoint, double processValue, TimeSpan ts)
        {
            // Ensure the timespan is not too small or zero.
            Ts = (ts.TotalSeconds >= TsMin) ? ts.TotalSeconds : TsMin;

            // Calculate rollup parameters
            K = 2 / Ts;
            b0 = Math.Pow(K, 2) * Kp + K * Ki + Ki * N + K * Kp * N + Math.Pow(K, 2) * Kd * N;
            b1 = 2 * Ki * N - 2 * Math.Pow(K, 2) * Kp - 2 * Math.Pow(K, 2) * Kd * N;
            b2 = Math.Pow(K, 2) * Kp - K * Ki + Ki * N - K * Kp * N + Math.Pow(K, 2) * Kd * N;
            a0 = Math.Pow(K, 2) + N * K;
            a1 = -2 * Math.Pow(K, 2);
            a2 = Math.Pow(K, 2) - K * N;

            // Age errors and output history
            e2 = e1;                        // Age errors one iteration
            e1 = e0;                        // Age errors one iteration
            e0 = setPoint - processValue;   // Compute new error
            y2 = y1;                        // Age outputs one iteration
            y1 = y0;                        // Age outputs one iteration
            y0 = -a1 / a0 * y1 - a2 / a0 * y2 + b0 / a0 * e0 + b1 / a0 * e1 + b2 / a0 * e2; // Calculate current output

            // Clamp output if needed
            if (y0 > OutputUpperLimit)
            {
                y0 = OutputUpperLimit;
            }
            else if (y0 < OutputLowerLimit)
            {
                y0 = OutputLowerLimit;
            }

            return y0;
        }

        /// <summary>
        /// Reset controller history effectively resetting the controller.
        /// </summary>
        public void ResetController()
        {
            e2 = 0;
            e1 = 0;
            e0 = 0;
            y2 = 0;
            y1 = 0;
            y0 = 0;
        }

        /// <summary>
        /// Proportional Gain, consider resetting controller if this parameter is drastically changed.
        /// </summary>
        public double Kp { get; set; }

        /// <summary>
        /// Integral Gain, consider resetting controller if this parameter is drastically changed.
        /// </summary>
        public double Ki { get; set; }

        /// <summary>
        /// Derivative Gain, consider resetting controller if this parameter is drastically changed.
        /// </summary>
        public double Kd { get; set; }

        /// <summary>
        /// Derivative filter coefficient.
        /// A smaller N for more filtering.
        /// A larger N for less filtering.
        /// Consider resetting controller if this parameter is drastically changed.
        /// </summary>
        public double N { get; set; }

        /// <summary>
        /// Minimum allowed sample period to avoid dividing by zero!
        /// The Ts value can be mistakenly set to too low of a value or zero on the first iteration.
        /// TsMin by default is set to 1 millisecond.
        /// </summary>
        public double TsMin { get; set; } = 0.001;

        /// <summary>
        /// Upper output limit of the controller.
        /// This should obviously be a numerically greater value than the lower output limit.
        /// </summary>
        public double OutputUpperLimit { get; set; }

        /// <summary>
        /// Lower output limit of the controller
        /// This should obviously be a numerically lesser value than the upper output limit.
        /// </summary>
        public double OutputLowerLimit { get; set; }
    }

}