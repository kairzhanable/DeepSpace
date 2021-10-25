using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;
using System;

public class NewBehaviourScript : MonoBehaviour
{
    double value = 0;

    public GameObject target;
    public GameObject ind;
    private Vector3 pos;
    void Start()
    {
        pos = ind.transform.position;
    }

    private double emaK = 0;

    void Update()
    {
        Vector3 local_diff_torque = calc( 1 );
        //local_diff_torque = ema_vector3(local_diff_torque, 0.5f);
        ind.transform.position = pos + local_diff_torque;
        Debug.Log(local_diff_torque);
        gameObject.GetComponent<Rigidbody>().AddTorque(local_diff_torque, ForceMode.Force);
    }

    /*private float x = 0;
    private float y = 0;
    private float z = 0;

    private Vector3 ema_vector3(Vector3 vector, float _alpfa){
        x = ema(vector.x, x, _alpfa);
        y = ema(vector.y, y, _alpfa);
        z = ema(vector.z, z, _alpfa);
        return new Vector3(x,y,z);

    }

    private float ema (float value, float ema, float _alpfa){
        return ema * (1 - _alpfa) + value * _alpfa;
    }*/

    private double old_angle = 0;
    private double old_speed = 0;
    private double old_torque = 0;

    private double ema_k = 0;
    public double alpfa = 0.001f;

    public double langle = 30f;
    public double soft = 5;

    private Vector3 calc(float max_torque){
        Quaternion target_rotation = target.transform.rotation;
        Quaternion current_rotation = gameObject.transform.rotation;
        Quaternion diff = target_rotation * Quaternion.Inverse(current_rotation);
        Vector3 angularVelocity = gameObject.GetComponent<Rigidbody>().angularVelocity;
        Vector3 local_diff_torque = new Vector3(diff.x, diff.y, diff.z);
        local_diff_torque = Vector3.Normalize(local_diff_torque);
        double angle = Quaternion.Angle(target_rotation, current_rotation);
        double speed = (old_angle - angle) / Time.deltaTime;
        double acceleration = (old_speed - speed) / Time.deltaTime;
        old_speed = speed;
        old_angle = angle;

        /*float expected_acceleration = (acceleration / old_torque);
        float max_available_torque = (expected_acceleration * max_torque);
        float distance = (speed * speed) / (2 * max_available_torque);
        float torque = 0;

        if(distance < angle){
            torque = max_torque;
        } else {
            torque = -max_torque;
        }
        old_torque = torque;
        return local_diff_torque * torque;*/

        double time = angle / Mathf.Abs((float)speed);
        if(speed > time){
            ema_k = ema_k * (1-alpfa) + alpfa;

        } else {
            ema_k = ema_k * (1-alpfa) - alpfa;
        }
        if(angle < langle){
            local_diff_torque += (angularVelocity * (float)(langle-angle)/(float)langle) / (float)soft;
        }
        /*if(angle < 1 && speed < 0.0001){
            gameObject.GetComponent<Rigidbody>().angularVelocity = new Vector3(0,0,0);
            gameObject.transform.rotation = target_rotation;
            return Vector3.zero;
        }*/
        return local_diff_torque * (float)ema_k;
    }
}
