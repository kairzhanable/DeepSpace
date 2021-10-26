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
        ind.transform.position = pos + local_diff_torque;
        Debug.Log(local_diff_torque);
        //gameObject.GetComponent<Rigidbody>().AddTorque(local_diff_torque, ForceMode.Force);
        gameObject.GetComponent<Rigidbody>().AddRelativeTorque(Quaternion.Inverse(transform.rotation) * local_diff_torque, ForceMode.Force);
    }

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

        double time = angle / Mathf.Abs((float)speed);
        if(speed/acceleration > time){
            ema_k = ema_k * (1-alpfa) + alpfa;

        } else {
            ema_k = ema_k * (1-alpfa) - alpfa;
        }
        if(angle < langle && angularVelocity.sqrMagnitude != 0f){
            local_diff_torque += (angularVelocity * (float)(langle-angle)/(float)langle) / (float)soft;
        }
        if(angle < 1 && speed < 0.1){
            gameObject.GetComponent<Rigidbody>().angularVelocity = new Vector3(0.0001f,0.0001f,0.0001f);
            return Vector3.zero;
        }
        return local_diff_torque * (float)ema_k;
    }
}
