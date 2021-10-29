using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
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

        void Awake() {
            rigidbody = gameObject.GetComponent<Rigidbody>();
            rcs = new MyRCS(rigidbody);
            qPID = new QPID(rigidbody);
            vPID = new VPID(rigidbody);
            IEngins = new SystemElements<IEngine>();
        }

        private void Start() {
        }

        public void ApplyManeur(Vector3 desired_speed, Quaternion desiredRotation){
            Vector3 desiredForce = vPID.calcForce(desired_speed);
            Vector3 desiredTorque = qPID.calcTorque(desiredRotation);

            //rigidbody.AddRelativeForce(desiredForce);
            //rigidbody.AddRelativeTorque(desiredTorque);

            if(desiredForce != Vector3.zero || desiredTorque != Vector3.zero){
                rcs.SetDesired(desiredForce, desiredTorque);
                rcs.Apply(IEngins);
            } else {
                for(int i = 0; i < IEngins.Length; i++){
                    IEngins.get(i).ApplyForce(0,0);
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
            if(!(module is IPhysical))
                return;
            IPhysical element = (IPhysical)module;
            element.SetRigidbody(rigidbody);
        }
    }

    public interface IControlled{
        //???
    }

    public interface IPhysical{
        void SetRigidbody(Rigidbody rigidbody);
    }

    public interface IEngine:IPhysical{
        void ApplyForce(float force_coefficient, float throttle_coefficient);
        void Recalculation();

        Vector3 max_force {get;}
        Vector3 max_torque {get;}

        Vector3 currentTorque {get;}
        Vector3 currentForce {get;}

        float force_coefficient {get;}
        float limit {get; set;}

        Vector3 possible_force(float coefficient);
        Vector3 possible_torque(float coefficient);
    }

    public class QPID{

        private Rigidbody rigidbody;
        private float max_Velocity = 1; 
        private PID pid;

        public QPID(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
            pid = new PID(5, 3, 3);
        }

        public Vector3 calcTorque(Quaternion targetRotation){
            Quaternion current_rotation = rigidbody.transform.rotation;
            Quaternion diff = targetRotation * Quaternion.Inverse(current_rotation);
            Vector3 angularVelocity = rigidbody.angularVelocity;
            Vector3 local_diff_torque = new Vector3(diff.x, diff.y, diff.z);
            Vector3 local_diff_torque_norm = Vector3.Normalize(local_diff_torque);
            float angle = Quaternion.Angle(targetRotation, current_rotation);

            if(local_diff_torque.magnitude < 0.0005){
                rigidbody.angularVelocity = Vector3.zero;
                return Vector3.zero;
            }

            Debug.Log(local_diff_torque.magnitude);

            float k = pid.GetOutput((float)angle, Time.fixedDeltaTime);

            return Quaternion.Inverse(rigidbody.transform.rotation) * (local_diff_torque_norm) * k;
        }
    }

    public class VPID{
        private Rigidbody rigidbody;

        public VPID(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        public Vector3 calcForce(Vector3 desired_speed){
            if((rigidbody.velocity - desired_speed).sqrMagnitude < 0.1){
                rigidbody.velocity = desired_speed;
                return Vector3.zero;
            }
            return (desired_speed - rigidbody.transform.InverseTransformDirection(rigidbody.velocity));
        }
    }

    public class PID
    {
        private float _p, _i, _d;
        private float _kp, _ki, _kd;
        private float _prevError;

        /// <summary>
        /// Constant proportion
        /// </summary>
        public float Kp
        {
            get
            {
                return _kp;
            }
            set
            {
                _kp = value;
            }
        }

        /// <summary>
        /// Constant integral
        /// </summary>
        public float Ki
        {
            get
            {
                return _ki;
            }
            set
            {
                _ki = value;
            }
        }

        /// <summary>
        /// Constant derivative
        /// </summary>
        public float Kd
        {
            get
            {
                return _kd;
            }
            set
            {
                _kd = value;
            }
        }

        public PID(float p, float i, float d)
        {
            _kp = p;
            _ki = i;
            _kd = d;
        }

        /// <summary>
        /// Based on the code from Brian-Stone on the Unity forums
        /// https://forum.unity.com/threads/rigidbody-lookat-torque.146625/#post-1005645
        /// </summary>
        /// <param name="currentError"></param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        public float GetOutput(float currentError, float deltaTime)
        {
            _p = currentError;
            _i += _p * deltaTime;
            _d = (_p - _prevError) / deltaTime;
            _prevError = currentError;
            
            return _p * Kp + _i * Ki + _d * Kd;
        }
    }

    public class MyRCS{
        private Rigidbody rigidbody;
        private Vector3 desiredForce;
        private Vector3 desiredTorque;
        private SystemElements<IEngine> IEngins;
        private List<float> coefs;

        public MyRCS(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
            coefs = new List<float>();
        }

        public void SetDesired(Vector3 desiredForce, Vector3 desiredTorque){
            this.desiredForce = desiredForce;
            this.desiredTorque = desiredTorque;
        }

        public void Apply(SystemElements<IEngine> IEngins){
            this.IEngins = IEngins;
            for(int i = 0; i < IEngins.Length; i++){
                IEngins.get(i).Recalculation();
            }
            refresh();
            float step = 0.5f;
            for(int iteration = 0; iteration < 200; iteration++){
                if(!optimize(step)){
                    step /= 2f;
                }
                if(CalcError() <= 1){
                    break;
                }
            }
            initEngine();
        }

        private void initEngine(){
            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                engine.ApplyForce(coefs[i], coefs[i]);
            }
        }

        private bool optimize(float step){

            float init_error = CalcError();
            int[] dirs = new int[] { 1, -1 };

            float min_error = init_error;
            bool optimized = false;
            int opt_engine = -1;
            float delta = 0;

            for(int i = 0; i < IEngins.Length; i++){
                foreach(int dir in dirs){
                    float new_error = vary(i, dir*step);
                    if(new_error < min_error){
                        min_error = new_error;
                        opt_engine = i;
                        delta = dir*step;
                        optimized = true;
                    }
                }
            }

            if(optimized){
                coefs[opt_engine] += delta;
            } 
            return optimized;
        }

        private float vary(int EngineID, float step){
            float cacheCoefficient = coefs[EngineID];/*IEngins.get(EngineID).force_coefficient;*/
            if(cacheCoefficient + step > 1 || cacheCoefficient + step < 0)
                return float.MaxValue;
            Vector3 newForce = IEngins.get(EngineID).possible_force(cacheCoefficient + step);
            Vector3 newTorque = IEngins.get(EngineID).possible_torque(cacheCoefficient + step);
            return CalcError(EngineID, newForce, newTorque);
        }

        private void refresh(){
            coefs.Clear();
            for(int i = 0; i < IEngins.Length; i++){
                coefs.Add(IEngins.get(i).force_coefficient);
            }
        }

        private float CalcError(){
            return CalcError(-1, Vector3.zero, Vector3.zero);
        }

        private float CalcError(int EngineID, Vector3 newForce, Vector3 newTorque){

            Vector3 errorLinear = Vector3.zero;
            Vector3 errorAngle = Vector3.zero;

            for(int i = 0; i < IEngins.Length; i++){
                if(i != EngineID){
                    errorLinear += IEngins.get(i).possible_force(coefs[i]);
                    errorAngle += IEngins.get(i).possible_torque(coefs[i]);
                } else {
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

        private void Calc(){

        }
    }

}

























/*
public class RCS{
        private Vector<double> initialGuess; // вектор начальных значений тяги ([0] * кол-во двиг.)
        private Vector3[] forces;            // вектор значений сил
        private Vector3[] torques;           // вектор значений моментов
        private Rigidbody rigidbody;
        private SystemElements<IEngine> IEngins;
        private Vector3 desiredForce;
        private Vector3 desiredTorque;

        private Vector<double> X1, X2, X3, X4, X5, X6;

        public RCS(Rigidbody rigidbody){
            this.rigidbody = rigidbody;
        }

        

        public void Refresh(SystemElements<IEngine> IEngins){
            this.IEngins = IEngins;
            forces = new Vector3[IEngins.Length];
            torques = new Vector3[IEngins.Length];
            initialGuess = Vector<double>.Build.Dense(IEngins.Length);
            X1 = Calc(Vector3.right, Vector3.zero);
            X2 = Calc(Vector3.up, Vector3.zero);
            X3 = Calc(Vector3.forward, Vector3.zero);
            X4 = Calc(Vector3.zero, Vector3.right);
            X5 = Calc(Vector3.zero, Vector3.up);
            X6 = Calc(Vector3.zero, Vector3.forward);
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

        private Vector<double> Calc(Vector3 desiredForce, Vector3 desiredTorque)
        {
            this.desiredForce = desiredForce;
            this.desiredTorque = desiredTorque;
            for (int i = 0; i < IEngins.Length; i++)
            {
                IEngine engine = IEngins.get(i);
                engine.Recalculation();
                this.forces[i] = engine.force;
                this.torques[i] = engine.Torque(1);
            }
            Func<Vector<double>, double> f_delegate = calculate;
            Vector<double> results = OfFunction(f_delegate, initialGuess);
            
            return results;
        }


        public void Apply(Vector3 desiredForce, Vector3 desiredTorque){
            double[][] biglist = new double[6][];
            biglist[0] = X1.ToArray();
            biglist[1] = X2.ToArray();
            biglist[2] = X3.ToArray();
            biglist[3] = X4.ToArray();
            biglist[4] = X5.ToArray();
            biglist[5] = X6.ToArray();
            Matrix<double> A = Matrix<double>.Build.DenseOfColumns(biglist);
            Matrix<double> b = Matrix<double>.Build.Dense(6, 1);
            /*var b = Vector<double>.Build.Dense(new double[] { desiredForce.x, desiredForce.y, desiredForce.z,
                                                              desiredTorque.x, desiredTorque.y, desiredTorque.z });*//*
            b[0, 0] = desiredForce.x;
            b[1, 0] = desiredForce.y;
            b[2, 0] = desiredForce.z;
            b[3, 0] = desiredTorque.x;
            b[4, 0] = desiredTorque.y;
            b[5, 0] = desiredTorque.z;

            var p = A.Transpose().PseudoInverse();
            var results_m = p * b;

            double[] results = new double[X1.Count];
            for(int i = 0; i < X1.Count; i++){
                results[i] = results_m[i,0];
            }
            
            //var results = A.Solve(b);

            /*Vector<double> results = Calc(desiredForce, desiredTorque);*//*
            for (int i = 0; i < IEngins.Length; i++)
            {
                float max = 0;
                foreach(float _result in results){
                    if(_result > max){
                        max = _result;
                    }
                }
                IEngine engine = IEngins.get(i);
                double result = results[i];
                engine.ApplyForce((float)result, (float)result / max);
            }
        }
    }

*/
