using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using System;
using System.Threading.Tasks;


public class RCS : MonoBehaviour
{
    [Tooltip("Вектор прилагаемого к кораблю усилия")]
    public Vector3 desiredForce;

    [Tooltip("Вектор прилагаемого к кораблю момента")]
    public Vector3 desiredTorque;

    [Tooltip("Список двигателей")]
    public GameObject[] trustersGO;

    [Tooltip("Масса корабля (фактор линейной инерции)")]
    public float mass;

    [Tooltip("Тензор инерции корабля (фактор инерции вращения)")]
    public Vector3 inertiaTensor;

    private Vector<double> initialGuess; // вектор начальных значений тяги ([0] * кол-во двиг.)
    private Vector3[] forces;            // вектор значений сил
    private Vector3[] torques;           // вектор значений моментов
    private Thruster[] thrusters;
    private Rigidbody rg;



    void Start()
    {
        rg = gameObject.GetComponent<Rigidbody>();
        rg.inertiaTensor = inertiaTensor;
        rg.mass = mass;
        initialGuess = Vector<double>.Build.Dense(trustersGO.Length);
        forces = new Vector3[trustersGO.Length];
        torques = new Vector3[trustersGO.Length];
        thrusters = new Thruster[trustersGO.Length];
        for (int i = 0; i < trustersGO.Length; i++)
        {
            Thruster thruster = trustersGO[i].GetComponent<Thruster>();
            thruster.Recalculation(rg);
            thrusters[i] = thruster;
        }
    }
    
    // минимизации ошибки функции
    private Vector<double> OfFunction(Func<Vector<double>, double> function, Vector<double> initialGuess, double tolerance = 1e-100, int maxIterations = 100000)
    {
        var objective = ObjectiveFunction.Value(function);
        var result = NelderMeadSimplex.Minimum(objective, initialGuess, tolerance, maxIterations);
        return result.MinimizingPoint;
    }

    // целевая функция
    private double calculate(Vector<double> coeff)
    {
        Vector3 totalForces = new Vector3(0, 0, 0);
        Vector3 totalTorques = new Vector3(0, 0, 0);
        double d = 0;                                                                       // фактор гашения
        for (int trusterId = 0; trusterId < trustersGO.Length; trusterId++)
        {
            totalForces += forces[trusterId] * (float)coeff[trusterId];
            totalTorques += torques[trusterId] * (float)coeff[trusterId];
            if (coeff[trusterId] > 1 || coeff[trusterId] < 0)
            {
                d += 999 + coeff[trusterId] * 999;
            }
        }
        Vector3 errorLinear = (totalForces - transform.rotation * desiredForce);
        Vector3 errorAngle = (totalTorques - transform.rotation * desiredTorque);
        return (double)(errorLinear.x * errorLinear.x +
                        errorLinear.y * errorLinear.y +
                        errorLinear.z * errorLinear.z +
                        errorAngle.x * errorAngle.x +
                        errorAngle.y * errorAngle.y +
                        errorAngle.z * errorAngle.z + d);
    }

    void Update()
    {
        for (int i = 0; i < thrusters.Length; i++)
        {
            Thruster thruster = thrusters[i];
            thruster.Recalculation(rg);
            forces[i] = thruster.force;
            torques[i] = thruster.torque;
        }

        Func<Vector<double>, double> f_delegate = calculate;
        Vector<double> results = OfFunction(f_delegate, initialGuess);

        for (int i = 0; i < thrusters.Length; i++)
        {
            Thruster thruster = thrusters[i];
            double result = results[i];
            thruster.addForce(gameObject.GetComponent<Rigidbody>(), (float)result);
        }
    }
}